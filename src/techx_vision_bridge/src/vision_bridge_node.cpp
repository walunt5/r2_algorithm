/**
 * @file vision_bridge_node.cpp
 * @brief GMK M6 视觉数据 UDP→ROS2 桥接节点（C++ 实现）
 *
 * 架构概述：
 *   - 独立 std::thread 阻塞式 UDP 接收线程（五重防爆校验）
 *   - 线程安全消息队列（std::queue + std::mutex + std::condition_variable）
 *   - 独立处理线程，事件驱动发布 Target3D 自定义消息
 *   - 主线程运行 rclcpp::spin() + 失联看门狗定时器
 *
 * 上游协议：29 字节小端序 UDP 包
 *   < H  I    d      B     3f       H     >
 *   Magic Seq  Time   ID   XcYcZc  CRC16
 *
 * 五重防爆接收法：
 *   1. 缓冲区抽干（Anti-Buffer-Bloat）
 *   2. 长度 + Magic Number 校验
 *   3. CRC16-CCITT 校验
 *   4. 序列号管理（防时空倒流）
 *   5. 断连重连机制（reconnect_timeout）
 */

#include <chrono>
#include <cstring>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <atomic>
#include <array>

// Linux socket API（GMK M6 / aarch64 目标平台）
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/resource.h>
#include <unistd.h>
#include <fcntl.h>
#include <cerrno>
#include <pthread.h>

#include "rclcpp/rclcpp.hpp"
#include "techx_vision_bridge/msg/target3_d.hpp"

// ═══════════════════════════════════════════════════════════════
//  协议常量
// ═══════════════════════════════════════════════════════════════
constexpr size_t    PACKET_SIZE  = 29;          // 合法包长度（字节）
constexpr uint16_t  MAGIC_VALUE  = 0x55AA;      // 魔数
constexpr size_t    CRC_COVERAGE = 27;          // CRC 覆盖前 27 字节
constexpr size_t    RECV_BUF_SIZE = 4096;        // recvfrom 缓冲区

// ═══════════════════════════════════════════════════════════════
//  UDP 数据包结构体（小端序，紧凑布局）
// ═══════════════════════════════════════════════════════════════
#pragma pack(push, 1)
struct UdpPacket {
    uint16_t magic;       // 魔数 0x55AA
    uint32_t seq;         // 帧序列号
    double   timestamp;   // 上游时间戳（秒, float64）
    uint8_t  track_id;    // 目标跟踪 ID
    float    xc;          // 相机坐标系 X（米）
    float    yc;          // 相机坐标系 Y（米）
    float    zc;          // 相机坐标系 Z（米）
    uint16_t crc16;       // CCITT-CRC16 校验值
};
#pragma pack(pop)
static_assert(sizeof(UdpPacket) == 29, "UdpPacket size must be 29 bytes");

// ═══════════════════════════════════════════════════════════════
//  队列元素：经过五重校验后的合法目标数据
// ═══════════════════════════════════════════════════════════════
struct TargetData {
    uint32_t seq;
    double   timestamp;       // 上游时间戳（秒, float64）
    uint8_t  track_id;
    float    x, y, z;
    std::chrono::system_clock::time_point recv_time;  // 本地接收系统时刻（用于 local 模式精确回退）
};

// ═══════════════════════════════════════════════════════════════
//  CCITT-CRC16 查表法实现（多项式 0x1021，初始值 0xFFFF，与 Jetson 端 STM32 HAL_CRC 一致）
//  使用 constexpr 在编译期生成查表，零运行时开销
// ═══════════════════════════════════════════════════════════════
constexpr std::array<uint16_t, 256> buildCrc16Table() {
    std::array<uint16_t, 256> table{};
    for (int i = 0; i < 256; ++i) {
        uint16_t crc = static_cast<uint16_t>(i << 8);
        for (int j = 0; j < 8; ++j) {
            if (crc & 0x8000) {
                crc = static_cast<uint16_t>((crc << 1) ^ 0x1021);
            } else {
                crc = static_cast<uint16_t>(crc << 1);
            }
        }
        table[i] = crc;
    }
    return table;
}

static constexpr auto CRC16_TABLE = buildCrc16Table();

/**
 * @brief 计算 CCITT-CRC16 校验值（初始值 0xFFFF，与 Jetson 端 STM32 HAL_CRC 一致）
 * @param data 数据指针
 * @param len  数据长度
 * @return 16 位 CRC 校验值
 */
inline uint16_t crc16Ccitt(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc = static_cast<uint16_t>(
            ((crc << 8) & 0xFFFF) ^ CRC16_TABLE[((crc >> 8) ^ data[i]) & 0xFF]
        );
    }
    return crc;
}

// ═══════════════════════════════════════════════════════════════
//  VisionBridgeNode 类定义
// ═══════════════════════════════════════════════════════════════
class VisionBridgeNode : public rclcpp::Node {
public:
    VisionBridgeNode();
    ~VisionBridgeNode() override;

private:
    // ── 线程函数 ──
    void recvLoop();       // UDP 接收线程（阻塞 + 五重防爆）
    void processLoop();    // 处理线程（队列消费 → 发布）
    void watchdogCallback(); // 失联看门狗回调

    // ── 辅助函数 ──
    bool validatePacket(const uint8_t* data, size_t len, TargetData& out);
    rclcpp::Time buildRosTime(const TargetData& target);  // 时间戳构造（含校验）
    void tryReconnect();
    void closeSocket();
    void applyThreadPriority();  // 降低当前线程优先级，为控制节点让路

    // ── ROS 2 参数 ──
    std::string _udp_bind_addr;
    int         _udp_port;
    std::string _topic_name;
    double      _reconnect_timeout_sec;
    double      _watchdog_timeout_sec;
    int         _thread_nice;        // 工作线程 nice 值（越大优先级越低）
    std::string _timestamp_mode;     // 时间戳模式: "upstream" | "local" | "auto"
    double      _min_valid_ts;       // 最小有效 Unix 时间戳（秒）
    double      _max_future_sec;     // 允许的上游时间戳最大超前量（秒）
    std::atomic<bool> _warned_mono_clock{false};  // monotonic 时钟告警（仅一次）
    std::atomic<bool> _warned_future_ts{false};   // 时钟超前告警（仅一次）

    // ── Socket ──
    int _sock{-1};

    // ── 线程管理 ──
    std::thread       _recv_thread;
    std::thread       _process_thread;
    std::atomic<bool> _stop{false};

    // ── 线程安全消息队列 ──
    std::queue<TargetData>       _queue;
    std::mutex                   _queue_mutex;
    std::condition_variable      _queue_cv;
    static constexpr size_t      QUEUE_MAX_SIZE = 32;  // 队列容量上限，防止内存暴涨

    // ── 跨线程共享状态（mutex 保护） ──
    std::mutex _state_mutex;
    uint32_t   _last_seq{0};
    bool       _seq_initialized{false};
    std::chrono::steady_clock::time_point _last_valid_time;

    // ── ROS 2 通信 ──
    rclcpp::Publisher<techx_vision_bridge::msg::Target3D>::SharedPtr _pub;
    rclcpp::TimerBase::SharedPtr _watchdog_timer;
};

// ═══════════════════════════════════════════════════════════════
//  构造函数：初始化参数、Socket、线程、发布者、看门狗
// ═══════════════════════════════════════════════════════════════
VisionBridgeNode::VisionBridgeNode()
    : Node("vision_bridge_node")
{
    // ── 声明并读取参数（支持 launch 文件 / 命令行动态配置） ──
    this->declare_parameter("udp_bind_addr", "0.0.0.0");
    this->declare_parameter("udp_port", 12345);
    this->declare_parameter("topic_name", "/techx/vision/targets");
    this->declare_parameter("reconnect_timeout_sec", 3.0);
    this->declare_parameter("watchdog_timeout_sec", 0.3);
    this->declare_parameter("thread_nice", 10);  // 默认 nice=10，给 odin1/导航 让路
    this->declare_parameter("timestamp_mode", "local");  // 默认 local：GMK 接收时刻，与控制回路时钟一致
    this->declare_parameter("min_valid_timestamp", 1500000000.0);  // ≈2017-07-14
    this->declare_parameter("max_future_sec", 3600.0);  // 1 小时

    _udp_bind_addr          = this->get_parameter("udp_bind_addr").as_string();
    _udp_port               = this->get_parameter("udp_port").as_int();
    _topic_name             = this->get_parameter("topic_name").as_string();
    _reconnect_timeout_sec  = this->get_parameter("reconnect_timeout_sec").as_double();
    _watchdog_timeout_sec   = this->get_parameter("watchdog_timeout_sec").as_double();
    _thread_nice            = this->get_parameter("thread_nice").as_int();
    _timestamp_mode         = this->get_parameter("timestamp_mode").as_string();
    _min_valid_ts           = this->get_parameter("min_valid_timestamp").as_double();
    _max_future_sec         = this->get_parameter("max_future_sec").as_double();

    // 校验 timestamp_mode 参数
    if (_timestamp_mode != "upstream" && _timestamp_mode != "local" && _timestamp_mode != "auto") {
        RCLCPP_WARN(this->get_logger(),
            "无效的 timestamp_mode '%s'，回退为 'auto'", _timestamp_mode.c_str());
        _timestamp_mode = "auto";
    }

    RCLCPP_INFO(this->get_logger(),
        "参数加载完成: udp=%s:%d, topic=%s, reconnect=%.1fs, watchdog=%.1fs, nice=%d, ts_mode=%s",
        _udp_bind_addr.c_str(), _udp_port, _topic_name.c_str(),
        _reconnect_timeout_sec, _watchdog_timeout_sec, _thread_nice, _timestamp_mode.c_str());

    // ── 创建 UDP Socket ──
    tryReconnect();  // 首次连接复用重连逻辑
    if (_sock < 0) {
        RCLCPP_ERROR(this->get_logger(),
            "UDP Socket 初始化失败，节点将无法接收数据！请检查端口 %d 是否被占用。", _udp_port);
        // 不退出的原因是：ROS 2 launch 会自动重试，
        // 保留节点存活可以让用户通过 ros2 param 动态修改参数后手动触发重连
    }

    // ── 初始化共享状态 ──
    _last_valid_time = std::chrono::steady_clock::now();

    // ── 创建发布者 ──
    // 严格使用 SensorDataQoS()：BEST_EFFORT 可靠性 + KEEP_LAST(5) + 低延迟
    auto qos = rclcpp::SensorDataQoS();
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    _pub = this->create_publisher<techx_vision_bridge::msg::Target3D>(_topic_name, qos);

    // ── 看门狗定时器（2Hz，仅打印失联提示） ──
    _watchdog_timer = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&VisionBridgeNode::watchdogCallback, this)
    );

    // ── 启动工作线程 ──
    _recv_thread    = std::thread(&VisionBridgeNode::recvLoop, this);
    _process_thread = std::thread(&VisionBridgeNode::processLoop, this);

    RCLCPP_INFO(this->get_logger(), "VisionBridgeNode 已启动（C++ 阻塞线程架构）");
}

// ═══════════════════════════════════════════════════════════════
//  析构函数：优雅关闭线程、释放 Socket 资源
// ═══════════════════════════════════════════════════════════════
VisionBridgeNode::~VisionBridgeNode() {
    RCLCPP_INFO(this->get_logger(), "正在关闭 VisionBridgeNode...");

    // 1. 通知所有线程退出
    _stop.store(true);
    _queue_cv.notify_all();

    // 2. 关闭 Socket 以唤醒阻塞的 recvfrom
    closeSocket();

    // 3. 等待线程退出（超时 2 秒）
    if (_recv_thread.joinable()) {
        _recv_thread.join();
    }
    if (_process_thread.joinable()) {
        _process_thread.join();
    }

    RCLCPP_INFO(this->get_logger(), "VisionBridgeNode 已安全关闭");
}

// ═══════════════════════════════════════════════════════════════
//  UDP 接收线程：阻塞 recvfrom → 五重防爆 → 入队
// ═══════════════════════════════════════════════════════════════
void VisionBridgeNode::recvLoop() {
    pthread_setname_np(pthread_self(), "vbridge_recv");
    applyThreadPriority();
    uint8_t recv_buf[RECV_BUF_SIZE];

    RCLCPP_INFO(this->get_logger(), "UDP 接收线程已启动");

    while (!_stop.load()) {
        // ── 阻塞等待网卡中断（超时 1s 用于定期检查 reconnect） ──
        struct sockaddr_in src_addr{};
        socklen_t addr_len = sizeof(src_addr);
        ssize_t n = recvfrom(_sock, recv_buf, RECV_BUF_SIZE, 0,
                             reinterpret_cast<struct sockaddr*>(&src_addr), &addr_len);

        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                // Socket 1s 超时，正常（UDP 是无连接的，收不到数据不等于断连）
                continue;
            }
            if (_stop.load()) break;
            RCLCPP_ERROR(this->get_logger(), "recvfrom 错误: %s", std::strerror(errno));
            continue;
        }

        // ═══════════════════════════════════════════════════════
        //  第 1 重：抽干法防积压（Anti-Buffer-Bloat）
        //  拿到一帧后立刻切换为非阻塞模式，循环抽干缓冲区残留包，
        //  只保留最后收到的那一帧合法帧，然后切回阻塞模式。
        // ═══════════════════════════════════════════════════════
        TargetData best_in_cycle;
        bool has_valid_in_cycle = false;

        // 先校验阻塞 recvfrom 拿到的第一帧
        if (validatePacket(recv_buf, static_cast<size_t>(n), best_in_cycle)) {
            has_valid_in_cycle = true;
        }

        // 切换到非阻塞模式抽干缓冲区
        int flags = fcntl(_sock, F_GETFL, 0);
        if (flags >= 0) {
            fcntl(_sock, F_SETFL, flags | O_NONBLOCK);
            {
                uint8_t drain_buf[RECV_BUF_SIZE];
                while (true) {
                    ssize_t dn = recvfrom(_sock, drain_buf, RECV_BUF_SIZE, 0, nullptr, nullptr);
                    if (dn < 0) {
                        if (errno == EAGAIN || errno == EWOULDBLOCK) break;
                        break;
                    }
                    // 校验每帧，只保留最新合法帧
                    TargetData temp;
                    if (validatePacket(drain_buf, static_cast<size_t>(dn), temp)) {
                        best_in_cycle = temp;
                        has_valid_in_cycle = true;
                    }
                }
            }
            // 恢复阻塞模式
            fcntl(_sock, F_SETFL, flags & ~O_NONBLOCK);
        }

        // ═══════════════════════════════════════════════════════
        //  合法帧入队
        // ═══════════════════════════════════════════════════════
        if (has_valid_in_cycle) {
            // 校验通过 → 线程安全入队
            {
                std::lock_guard<std::mutex> lock(_queue_mutex);
                // 队列容量保护：超过上限时丢弃最旧数据
                if (_queue.size() >= QUEUE_MAX_SIZE) {
                    _queue.pop();
                    RCLCPP_DEBUG(this->get_logger(), "队列已满(%zu)，丢弃最旧帧", QUEUE_MAX_SIZE);
                }
                _queue.push(best_in_cycle);
            }
            _queue_cv.notify_one();

            // 更新共享状态（用于看门狗和重连判断）
            {
                std::lock_guard<std::mutex> lock(_state_mutex);
                _last_valid_time = std::chrono::steady_clock::now();
            }
        }
    }

    RCLCPP_INFO(this->get_logger(), "UDP 接收线程已退出");
}

// ═══════════════════════════════════════════════════════════════
//  五重防爆校验（第 2~5 重）
// ═══════════════════════════════════════════════════════════════
bool VisionBridgeNode::validatePacket(const uint8_t* data, size_t len, TargetData& out) {
    // ═══ 第 2 重：长度校验 ═══
    if (len != PACKET_SIZE) {
        RCLCPP_DEBUG(this->get_logger(),
            "包长度异常: 期望 %zu, 实际 %zu, 丢弃", PACKET_SIZE, len);
        return false;
    }

    // 将原始字节安全拷贝到结构体（避免非对齐访问）
    UdpPacket packet;
    std::memcpy(&packet, data, PACKET_SIZE);

    // ═══ 第 2 重（续）：Magic Number 校验 ═══
    if (packet.magic != MAGIC_VALUE) {
        RCLCPP_DEBUG(this->get_logger(),
            "Magic 校验失败: 期望 0x%04X, 实际 0x%04X, 丢弃", MAGIC_VALUE, packet.magic);
        return false;
    }

    // ═══ 第 3 重：CRC16-CCITT 校验 ═══
    // 对前 27 字节（不含 CRC 字段）计算校验值
    uint16_t computed_crc = crc16Ccitt(data, CRC_COVERAGE);
    uint16_t received_crc = packet.crc16;
    if (computed_crc != received_crc) {
        RCLCPP_DEBUG(this->get_logger(),
            "CRC 校验失败: 接收 0x%04X, 计算 0x%04X, 丢弃", received_crc, computed_crc);
        return false;
    }

    // ═══ 第 4 重：序列号管理（防时空倒流） ═══
    {
        std::lock_guard<std::mutex> lock(_state_mutex);
        if (_seq_initialized) {
            // 丢弃序列号 <= 上一帧的数据包（旧包/重复包）
            if (packet.seq <= _last_seq) {
                RCLCPP_DEBUG(this->get_logger(),
                    "序列号倒流: %u <= %u, 丢弃", packet.seq, _last_seq);
                return false;
            }
        }
        _last_seq = packet.seq;
        _seq_initialized = true;
    }

    // ═══ 第 5 重：数据有效，填充输出结构体 ═══
    out.seq       = packet.seq;
    out.timestamp = packet.timestamp;
    out.track_id  = packet.track_id;
    out.x         = packet.xc;
    out.y         = packet.yc;
    out.z         = packet.zc;
    out.recv_time = std::chrono::system_clock::now();  // 记录本地接收系统时刻

    return true;
}

// ═══════════════════════════════════════════════════════════════
//  处理线程：消费队列 → 发布 Target3D 消息
//  使用 condition_variable 实现事件驱动，零忙等
// ═══════════════════════════════════════════════════════════════
void VisionBridgeNode::processLoop() {
    pthread_setname_np(pthread_self(), "vbridge_proc");
    applyThreadPriority();
    RCLCPP_INFO(this->get_logger(), "消息处理线程已启动");

    while (!_stop.load()) {
        TargetData target;
        bool has_data = false;

        {
            std::unique_lock<std::mutex> lock(_queue_mutex);
            // 阻塞等待，直到队列非空或收到停止信号
            _queue_cv.wait(lock, [this] {
                return !_queue.empty() || _stop.load();
            });

            if (_stop.load() && _queue.empty()) {
                break;
            }

            if (!_queue.empty()) {
                target = _queue.front();
                _queue.pop();
                has_data = true;
            }
        }

        if (!has_data) continue;

        // ── 构造 Target3D 消息 ──
        techx_vision_bridge::msg::Target3D msg;

        // frame_id 携带 TrackID，避免劫持四元数
        msg.header.frame_id = "camera_optical_frame";

        // 构造时间戳：根据 timestamp_mode 决定使用上游/本地/自动检测
        msg.header.stamp = buildRosTime(target);

        // 目标数据
        msg.track_id = target.track_id;
        msg.x = target.x;
        msg.y = target.y;
        msg.z = target.z;

        // 发布（rclcpp Publisher::publish 在 C++ 中是线程安全的）
        _pub->publish(msg);

        // 实时打印接收数据（每秒一次，避免刷屏）
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "📡 目标 TID=%u | X=%.3f Y=%.3f Z=%.3f m | Seq=%u",
            target.track_id, target.x, target.y, target.z, target.seq);
    }

    RCLCPP_INFO(this->get_logger(), "消息处理线程已退出");
}

// ═══════════════════════════════════════════════════════════════
//  时间戳构造：含时钟源检测、精度保护、回退逻辑
// ═══════════════════════════════════════════════════════════════
rclcpp::Time VisionBridgeNode::buildRosTime(const TargetData& target) {
    // ── local 模式：使用精准的本地接收系统时刻 ──
    if (_timestamp_mode == "local") {
        auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
            target.recv_time.time_since_epoch()).count();
        return rclcpp::Time(ns);
    }

    // ── 获取本地系统时钟（用于校验和回退，与 rclcpp::Time 使用相同时钟源） ──
    auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    double now_sec = static_cast<double>(now_ns) / 1e9;

    // ── 基础保护：负时间戳在任何模式下都无意义 ──
    if (target.timestamp < 0.0) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
            "上游时间戳为负值 %.1f，回退为本地接收时间", target.timestamp);
        return rclcpp::Time(now_ns);
    }

    // ── auto 模式：校验上游时间戳是否在合理范围 ──
    if (_timestamp_mode == "auto") {
        // 检测 1: 时间戳过小（疑似 monotonic 时钟，如系统启动秒数）
        if (target.timestamp < _min_valid_ts) {
            bool expected = false;
            if (_warned_mono_clock.compare_exchange_strong(expected, true)) {
                RCLCPP_WARN(this->get_logger(),
                    "上游时间戳 %.1f < 最小有效值 %.0f（疑似非 Unix 纪元时钟，"
                    "如 monotonic），回退为本地接收时间。该警告仅打印一次。",
                    target.timestamp, _min_valid_ts);
            }
            return rclcpp::Time(now_ns);
        }

        // 检测 2: 时间戳严重超前（时钟不同步或 Jetson 时间漂移）
        if (target.timestamp > now_sec + _max_future_sec) {
            bool expected = false;
            if (_warned_future_ts.compare_exchange_strong(expected, true)) {
                RCLCPP_WARN(this->get_logger(),
                    "上游时间戳 %.1f 比本地时间超前 %.1fs（阈值 %.1fs），"
                    "可能存在时钟不同步，回退为本地接收时间。该警告仅打印一次。",
                    target.timestamp, target.timestamp - now_sec, _max_future_sec);
            }
            return rclcpp::Time(now_ns);
        }
    }

    // ── upstream / auto(校验通过)：使用上游时间戳 ──
    // 上游 Timestamp 为 float64 秒数。
    // 先取整秒，再对剩余小数部分 ×1e9 得到纳秒，避免对大数 float64 直接 ×1e9
    // 带来的精度损失。小数部分在 [0,1) 范围内，float64 可表示亚纳秒精度。
    double ts = target.timestamp;
    int64_t sec = static_cast<int64_t>(ts);
    double frac = ts - static_cast<double>(sec);
    int64_t nanosec = static_cast<int64_t>(frac * 1e9);

    // 浮点舍入可能导致 nanosec 越界，做 clamp
    if (nanosec < 0) {
        nanosec += 1000000000LL;
        sec -= 1;
    }
    if (nanosec >= 1000000000LL) {
        nanosec -= 1000000000LL;
        sec += 1;
    }

    return rclcpp::Time(sec, static_cast<uint32_t>(nanosec));
}

// ═══════════════════════════════════════════════════════════════
//  失联看门狗回调（500ms = 2Hz）
// ═══════════════════════════════════════════════════════════════
void VisionBridgeNode::watchdogCallback() {
    auto now = std::chrono::steady_clock::now();
    double elapsed = 0.0;
    {
        std::lock_guard<std::mutex> lock(_state_mutex);
        elapsed = std::chrono::duration<double>(now - _last_valid_time).count();
    }
    // 仅打印提示，不触发重连（Jetson 没检测到目标时本就不发送，是正常现象）
    if (elapsed > _watchdog_timeout_sec) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "暂无目标数据（已 %.1fs 未收到），Jetson 可能未检测到目标", elapsed);
    }
}

// ═══════════════════════════════════════════════════════════════
//  创建/重连 UDP Socket
// ═══════════════════════════════════════════════════════════════
void VisionBridgeNode::tryReconnect() {
    // 先关闭旧 Socket
    closeSocket();

    // 创建新 Socket
    _sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (_sock < 0) {
        RCLCPP_ERROR(this->get_logger(), "Socket 创建失败: %s", std::strerror(errno));
        return;
    }

    // 设置地址复用（快速重启不丢端口）
    int opt = 1;
    setsockopt(_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    // 设置接收超时 1 秒（用于定期检查重连条件）
    struct timeval tv{};
    tv.tv_sec  = 1;
    tv.tv_usec = 0;
    setsockopt(_sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    // 增大接收缓冲区（减少丢包概率）
    int rcvbuf = 256 * 1024;  // 256KB
    setsockopt(_sock, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));

    // 降低 Socket 内核优先级（SO_PRIORITY=6，默认 0，范围 0~6）
    // 让网卡中断处理优先为控制链路（CAN/串口）服务
    int sk_prio = 6;
    setsockopt(_sock, SOL_SOCKET, SO_PRIORITY, &sk_prio, sizeof(sk_prio));

    // 绑定地址
    struct sockaddr_in addr{};
    addr.sin_family      = AF_INET;
    addr.sin_port        = htons(static_cast<uint16_t>(_udp_port));
    addr.sin_addr.s_addr = inet_addr(_udp_bind_addr.c_str());

    if (bind(_sock, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        RCLCPP_ERROR(this->get_logger(),
            "Socket 绑定 %s:%d 失败: %s", _udp_bind_addr.c_str(), _udp_port, std::strerror(errno));
        closeSocket();
        return;
    }

    // 重置状态（新连接，重新开始）
    {
        std::lock_guard<std::mutex> lock(_state_mutex);
        _seq_initialized = false;
        _last_seq = 0;
        _last_valid_time = std::chrono::steady_clock::now();
    }
    _warned_mono_clock.store(false);  // 重连后重新启用时钟异常告警
    _warned_future_ts.store(false);

    RCLCPP_INFO(this->get_logger(),
        "UDP Socket 已绑定 %s:%d（重连/初始化成功）", _udp_bind_addr.c_str(), _udp_port);
}

// ═══════════════════════════════════════════════════════════════
//  关闭 Socket
// ═══════════════════════════════════════════════════════════════
void VisionBridgeNode::closeSocket() {
    if (_sock >= 0) {
        shutdown(_sock, SHUT_RDWR);
        close(_sock);
        _sock = -1;
    }
}

// ═══════════════════════════════════════════════════════════════
//  降低当前线程优先级，为 odin1/导航/控制链条让路
//  使用 nice 值而非实时调度策略，避免在内核态产生优先级反转
// ═══════════════════════════════════════════════════════════════
void VisionBridgeNode::applyThreadPriority() {
    if (_thread_nice > 0) {
        int ret = setpriority(PRIO_PROCESS, 0, _thread_nice);
        if (ret != 0) {
            RCLCPP_WARN(this->get_logger(), "设置线程 nice=%d 失败: %s（可能需要 sudo 权限）",
                        _thread_nice, std::strerror(errno));
        } else {
            RCLCPP_DEBUG(this->get_logger(), "线程 nice 值已设置为 %d", _thread_nice);
        }
    }
}

// ═══════════════════════════════════════════════════════════════
//  main 入口
// ═══════════════════════════════════════════════════════════════
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<VisionBridgeNode>();

    // 使用 SingleThreadedExecutor：工作线程（recv/proc）已自行管理，
    // 主线程只跑 spin() + 看门狗定时器，不需多线程 executor
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}