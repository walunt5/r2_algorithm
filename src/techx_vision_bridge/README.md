# techx_vision_bridge — 视觉数据 UDP→ROS2 桥接节点

## 目录

- [1. 项目简介](#1-项目简介)
- [2. 系统架构](#2-系统架构)
- [3. 通信协议](#3-通信协议)
- [4. 目录结构](#4-目录结构)
- [5. 环境要求](#5-环境要求)
- [6. 快速开始（PC 实验）](#6-快速开始pc-实验)
  - [6.1 网络配置](#61-网络配置)
  - [6.2 编译](#62-编译)
  - [6.3 启动接收节点（PC）](#63-启动接收节点pc)
  - [6.4 方式一：用 test_sender.py 模拟发送](#64-方式一用-test_senderpy-模拟发送)
  - [6.5 方式二：接收真实 Jetson TECHX_vision 数据](#65-方式二接收真实-jetson-techx_vision-数据)
  - [6.6 验证接收](#66-验证接收)
- [7. 参数配置说明](#7-参数配置说明)
- [8. 如何订阅数据（在你的代码中）](#8-如何订阅数据在你的代码中)
  - [8.1 Python 订阅](#81-python-订阅)
  - [8.2 C++ 订阅](#82-c-订阅)
- [9. 移植到 GMK 机器人](#9-移植到-gmk-机器人)
- [10. 常见问题排查](#10-常见问题排查)
- [11. 五重防爆校验详解](#11-五重防爆校验详解)

---

## 1. 项目简介

`techx_vision_bridge` 是一个 ROS 2 (Humble) 功能包，用于**从以太网口接收上游视觉系统（如 Jetson Orin NX）的 UDP 数据包，经五重防爆校验后，转化为自定义 Target3D 消息发布到 ROS 2 话题**。

| 属性 | 值 |
|------|-----|
| 版本 | 2.0.0 |
| 语言 | C++17 |
| 目标平台 | x86_64 (PC 开发) / aarch64 (GMK M6 嵌入式) |
| ROS 2 发行版 | Humble |
| 许可 | MIT |

---

## 2. 系统架构

```
┌─────────────────────┐         UDP (29 bytes)          ┌──────────────────────┐
│   上游视觉系统        │ ───────────────────────────────> │   vision_bridge_node  │
│  (Jetson Orin NX)   │         以太网直连               │   (你的 PC / GMK)     │
│                     │                                  │                       │
│  test_sender.py     │                                  │  三线程架构：          │
│  (模拟发送)          │                                  │  • 接收线程 (阻塞)     │
└─────────────────────┘                                  │  • 处理线程 (事件驱动)  │
                                                         │  • 主线程 (看门狗)     │
                                                         └──────────┬───────────┘
                                                                    │
                                                         ROS 2 Topic:
                                                         /techx/vision/targets
                                                                    │
                                                         ┌──────────▼───────────┐
                                                         │   你的订阅节点         │
                                                         │  (控制/决策/可视化)    │
                                                         └──────────────────────┘
```

### 三线程模型

| 线程 | 职责 | 驱动方式 |
|------|------|----------|
| 主线程 | 运行 `rclcpp::spin()` + 10Hz 看门狗定时器 | Executor 调度 |
| 接收线程 | 阻塞式 `recvfrom` → 五重防爆校验 → 入队 | 网卡中断驱动 |
| 处理线程 | 队列消费 → 构造 Target3D 消息 → 发布 | 条件变量事件驱动 |

线程间通过 `std::queue<TargetData>` + `std::mutex` + `std::condition_variable` 通信，队列容量上限 32，防止内存暴涨。

---

## 3. 通信协议

上游视觉系统发送 **29 字节小端序 UDP 包**，结构如下：

```
偏移      0        2        6             14       15       19       23       27      29
      ┌────────┬────────┬──────────────┬───────┬───────┬───────┬───────┬────────┬──────┐
      │ Magic  │  Seq   │  Timestamp   │  ID   │  Xc   │  Yc   │  Zc   │ CRC16  │      │
      │ uint16 │ uint32 │   float64    │ uint8 │float32│float32│float32│ uint16 │      │
      └────────┴────────┴──────────────┴───────┴───────┴───────┴───────┴────────┴──────┘
       ←──────────────────────── 27 bytes (CRC coverage) ────────────────────────→
```

| 字段 | 类型 | 字节数 | 说明 |
|------|------|--------|------|
| `magic` | `uint16_t` | 2 | 魔数，固定为 `0x55AA`，用于快速识别合法包 |
| `seq` | `uint32_t` | 4 | 帧序列号，递增，用于防重复/倒流 |
| `timestamp` | `float64` | 8 | 上游时间戳（秒），用于多传感器时间同步 |
| `track_id` | `uint8_t` | 1 | 目标跟踪 ID，取值范围 0~255 |
| `xc` | `float32` | 4 | 相机坐标系 X 坐标（米） |
| `yc` | `float32` | 4 | 相机坐标系 Y 坐标（米） |
| `zc` | `float32` | 4 | 相机坐标系 Z 坐标（米） |
| `crc16` | `uint16_t` | 2 | CCITT-CRC16 校验值，覆盖前 27 字节 |

### 自定义 ROS 2 消息：Target3D

```cpp
std_msgs/Header header    // 时间戳 + frame_id（frame_id 格式: "camera_link_<track_id>"）
uint8  track_id           // 目标跟踪 ID
float32 x                 // 相机坐标系 X（米）
float32 y                 // 相机坐标系 Y（米）
float32 z                 // 相机坐标系 Z（米）
```

> **重要**：`header.stamp` 使用上游原始时间戳，而非本地系统时间，这对多传感器时间同步至关重要。
> `frame_id` 被设计为 `camera_link_<track_id>` 格式，利用 frame_id 携带 TrackID，避免劫持四元数字段。

---

## 4. 目录结构

```
GMK/
├── README.md                          # 你正在阅读的文档
├── test_sender.py                     # 模拟发送脚本（在 Jetson 上运行）
└── src/
    └── techx_vision_bridge/
        ├── CMakeLists.txt             # 构建配置
        ├── package.xml                # 包元信息
        ├── config/
        │   └── vision_bridge.yaml     # 参数配置文件（改端口/超时在这里）
        ├── launch/
        │   └── vision_bridge.launch.py # 启动文件（一键启动）
        ├── msg/
        │   └── Target3D.msg           # 自定义消息定义
        └── src/
            └── vision_bridge_node.cpp # 核心节点实现（约 550 行）
```

---

## 5. 环境要求

| 软件 | 版本要求 |
|------|----------|
| 操作系统 | Ubuntu 22.04 |
| ROS 2 | Humble (桌面完整安装) |
| CMake | >= 3.8 |
| GCC / Clang | 支持 C++17 |
| Python | >= 3.8 |

### 安装 ROS 2 Humble（如未安装）

```bash
# 设置 locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# 添加 ROS 2 源
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 安装
sudo apt update
sudo apt install ros-humble-desktop
```

---

## 6. 快速开始（PC 实验）

以下步骤演示如何在 PC 上接收来自 Jetson Orin NX 的 UDP 视觉数据。

### 拓扑结构

```
[Jetson Orin NX]  ────以太网线直连────>  [你的 PC]
   192.168.10.101                         192.168.10.100
   跑 TECHX_vision                        跑 vision_bridge_node
   (发送真实视觉数据)                      (接收 + 发布 ROS2 话题)
```

> **说明**：以下 IP 使用 `192.168.10.x` 网段，与 Jetson 端 `DEPLOY_GUIDE.md` 中的默认配置一致。
> 如果你的 Jetson 上 `config.json` 里的 `target_ip` 是其他 IP，请相应调整。

### 6.1 网络配置

**只需要改 IP 地址，无需改任何代码。**

#### 在 PC 上（接收端）

```bash
# 1. 查看网口名
ip link show

# 2. 给网口配置静态 IP（假设网口名为 enp3s0）
sudo ip addr add 192.168.10.100/24 dev enp3s0
sudo ip link set enp3s0 up

# 3. 验证
ip addr show enp3s0
```

> **永久配置（可选）**：编辑 `/etc/netplan/01-vision.yaml`，然后 `sudo netplan apply`。
> 详见本文档末尾的 [附录 A：永久网络配置](#附录-a永久网络配置)。

#### 在 Jetson Orin NX 上（发送端）

```bash
# 配置网口 IP
sudo ip addr add 192.168.10.101/24 dev eth0
sudo ip link set eth0 up

# 验证与 PC 的连通性
ping 192.168.10.100
```

#### 防火墙放行（如有）

```bash
# 在 PC 上放行 UDP 端口
sudo ufw allow 12345/udp
```

### 6.2 编译

```bash
cd /home/cyrus/robocon2026/GMK
colcon build --packages-select techx_vision_bridge
source install/setup.bash
```

每次打开新终端都需要重新 `source`，或者把下面这行加到 `~/.bashrc`：

```bash
echo "source /home/cyrus/robocon2026/GMK/install/setup.bash" >> ~/.bashrc
```

### 6.3 启动接收节点（PC）

用 launch 文件一键启动，**所有参数自动从 config/vision_bridge.yaml 加载**：

```bash
ros2 launch techx_vision_bridge vision_bridge.launch.py
```

终端输出：

```
[INFO] [vision_bridge_node]: 参数加载完成: udp=0.0.0.0:12345, topic=/techx/vision/targets, reconnect=3.0s, watchdog=0.3s
[INFO] [vision_bridge_node]: UDP Socket 已绑定 0.0.0.0:12345（重连/初始化成功）
[INFO] [vision_bridge_node]: VisionBridgeNode 已启动（C++ 阻塞线程架构）
[INFO] [vision_bridge_node]: UDP 接收线程已启动
[INFO] [vision_bridge_node]: 消息处理线程已启动
```

### 6.4 方式一：用 test_sender.py 模拟发送

`test_sender.py` 是一个独立的 Python 脚本，**不依赖 ROS 2**。用于在没有真实相机数据时，模拟 Jetson 发送测试数据。

**修改 `test_sender.py` 第 21 行的目标 IP**（只需改这一处）：

```python
SEND_TO_IP   = "192.168.10.100"   # ← 改成 PC 的网口 IP
SEND_TO_PORT = 12345              # 与接收端配置一致
SEND_RATE_HZ = 30                # 发包频率
TARGET_COUNT = 1                 # 模拟目标数量
```

然后在 Jetson（或任何一台能 ping 通 PC 的机器）上运行：

```bash
python3 test_sender.py
```

输出示例：

```
test_sender 启动
  → 目标地址: 192.168.10.100:12345
  → 发包频率: 30 Hz
  → 目标数量: 1
  → 魔数:     0x55AA

[OK] 已发送 60 帧, 最新序列号=59, 频率~30Hz
```

### 6.5 方式二：接收真实 Jetson TECHX_vision 数据

> 前提：Jetson 上已经按照 `DEPLOY_GUIDE.md` 部署了 TECHX_vision 视觉程序。

**在 Jetson 上：**

1. 确认 `config.json` 中的目标 IP 指向你的 PC：

   ```json
   {
       "udp": {
           "target_ip": "192.168.10.100",
           "target_port": 12345
       }
   }
   ```

2. 启动视觉程序：

   ```bash
   cd TECHX_vision
   ./start_jetson.sh
   ```

3. Jetson 启动后，只要检测到目标且深度有效（Zc > 0），就会自动发 UDP 包到你的 PC。

**无需在 PC 端做任何额外配置** —— `vision_bridge_node` 监听 `0.0.0.0:12345`，直接就能收到。

> **重要提示**：Jetson 端 CRC16 初始值是 `0xFFFF`（与 STM32 HAL_CRC 一致），GMK 端（本项目）已同步为 `0xFFFF`，校验不会失败。

### 6.6 验证接收

在 PC 上另开终端，监听话题：

```bash
source install/setup.bash
ros2 topic echo /techx/vision/targets
```

正常接收时会看到：

```
header:
  stamp:
    sec: 1700
    nanosec: 123456789
  frame_id: camera_link_0
track_id: 0
x: 1.234
y: -0.156
z: 2.010
---
```

**其他常用命令：**

```bash
# 查看话题列表
ros2 topic list

# 查看话题信息
ros2 topic info /techx/vision/targets

# 查看消息类型
ros2 interface show techx_vision_bridge/msg/Target3D

# 查看发布频率
ros2 topic hz /techx/vision/targets
```

---

## 7. 参数配置说明

所有参数集中在 `config/vision_bridge.yaml` 中管理，**一个文件搞定全部配置**。

```yaml
techx_vision_bridge:
  ros__parameters:
    # --- 网口设置 ---
    # 监听地址。"0.0.0.0" = 所有网口；指定 IP = 只监听该网口
    udp_bind_addr: "0.0.0.0"

    # 监听端口（必须和 Jetson 发送端一致）
    udp_port: 12345

    # --- 发布话题 ---
    topic_name: "/techx/vision/targets"

    # --- 超时设置 ---
    # 连续多少秒没收到合法数据 → 自动重连 Socket
    reconnect_timeout_sec: 3.0

    # 连续多少秒没收到合法数据 → 打印"信号丢失"警告
    # 应小于 reconnect_timeout，起提前预警作用
    watchdog_timeout_sec: 0.3
```

| 参数 | 类型 | 默认值 | 含义 |
|------|------|--------|------|
| `udp_bind_addr` | string | `0.0.0.0` | 监听网口地址，`0.0.0.0` 表示所有网口 |
| `udp_port` | int | `12345` | 监听端口，需与发送端一致 |
| `topic_name` | string | `/techx/vision/targets` | 发布话题名 |
| `reconnect_timeout_sec` | double | `3.0` | 超时重连阈值（秒），超时自动重建 Socket |
| `watchdog_timeout_sec` | double | `0.3` | 看门狗告警阈值（秒），超时打印 WARN 日志 |

**修改配置后，重新编译即可生效：**

```bash
colcon build --packages-select techx_vision_bridge
```

---

## 8. 如何订阅数据（在你的代码中）

### 8.1 Python 订阅

```python
import rclpy
from rclpy.node import Node
from techx_vision_bridge.msg import Target3D

class VisionSubscriber(Node):
    def __init__(self):
        super().__init__('vision_subscriber')

        # 使用 SensorDataQoS 匹配发布端的 QoS 策略
        self.subscription = self.create_subscription(
            Target3D,
            '/techx/vision/targets',
            self.callback,
            rclpy.qos.SensorDataQoS()
        )

    def callback(self, msg: Target3D):
        """收到每帧数据时被调用"""
        self.get_logger().info(
            f'目标 {msg.track_id}: '
            f'x={msg.x:.3f}m, y={msg.y:.3f}m, z={msg.z:.3f}m, '
            f't={msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}'
        )

def main():
    rclpy.init()
    node = VisionSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 8.2 C++ 订阅

```cpp
#include <rclcpp/rclcpp.hpp>
#include <techx_vision_bridge/msg/target3_d.hpp>

class VisionSubscriber : public rclcpp::Node {
public:
    VisionSubscriber() : Node("vision_subscriber") {
        // 使用 SensorDataQoS 匹配发布端的 QoS 策略
        subscription_ = this->create_subscription<
            techx_vision_bridge::msg::Target3D>(
            "/techx/vision/targets",
            rclcpp::SensorDataQoS(),
            std::bind(&VisionSubscriber::callback, this, std::placeholders::_1)
        );
    }

private:
    void callback(const techx_vision_bridge::msg::Target3D::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(),
            "目标 %d: x=%.3f, y=%.3f, z=%.3f, t=%d.%09d",
            msg->track_id, msg->x, msg->y, msg->z,
            msg->header.stamp.sec, msg->header.stamp.nanosec);
    }

    rclcpp::Subscription<techx_vision_bridge::msg::Target3D>::SharedPtr subscription_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VisionSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

> **关键提示**：订阅端必须使用 **`rclcpp::SensorDataQoS()`**（或 `rclpy.qos.SensorDataQoS()`），
> 因为发布端使用的是 `BEST_EFFORT` 可靠性策略。如果订阅端使用默认的 `RELIABLE` QoS，将收不到消息。

---

## 9. 移植到 GMK 机器人

从 PC 移植到 GMK M6（aarch64 嵌入式平台）只需两步：

### 9.1 交叉编译（在 PC 上为 aarch64 编译）

```bash
cd /home/cyrus/robocon2026/GMK
colcon build --packages-select techx_vision_bridge \
    --cmake-args \
    -DCMAKE_TOOLCHAIN_FILE=/path/to/aarch64-toolchain.cmake
```

> 具体 toolchain 文件路径取决于你的交叉编译 SDK 配置。

### 9.2 或直接在 GMK 上本地编译

将 `GMK/` 目录拷贝到 GMK 机器人上，然后：

```bash
cd ~/GMK
colcon build --packages-select techx_vision_bridge
source install/setup.bash
ros2 launch techx_vision_bridge vision_bridge.launch.py
```

配置文件 `config/vision_bridge.yaml` 无需修改（`udp_bind_addr: "0.0.0.0"` 监听所有网口），
只需在 GMK 上**配置好网口 IP** 即可。

---

## 10. 常见问题排查

| 现象 | 可能原因 | 排查方法 |
|------|----------|----------|
| 编译报错 `找不到 rclcpp` | 未安装 ROS 2 或未 source | `source /opt/ros/humble/setup.bash` |
| `ros2 launch` 找不到包 | 未 source 工作空间 | `source install/setup.bash` |
| 节点启动后无任何输出 | 上游未发包 | 在 Jetson 上运行 `test_sender.py` |
| 节点启动后显示"信号丢失" | 网络不通或包格式错误 | `ping` 检查连通性，确认包格式 29 字节 |
| `ros2 topic echo` 无输出 | QoS 不匹配 | 确认订阅端使用 `SensorDataQoS()` |
| 偶尔有数据，但不稳定 | 防火墙或网线问题 | 检查 `sudo ufw status`，重新插拔网线 |
| 大量 `CRC 校验失败` 日志 | 发送端 CRC 算法不一致 | 确认发送端使用 CCITT-CRC16，多项式 0x1021 |
| 大量 `Magic 校验失败` 日志 | 发送端包格式错误 | 确认 Magic 字段为 `0x55AA` 小端序 |
| 大量 `序列号倒流` 日志 | 上游重启或发包异常 | 正常现象，节点会自动处理 |
| 数据有延迟 | 队列积压 | 检查 `SEND_RATE_HZ` 是否过高 |

---

## 11. 五重防爆校验详解

| 重数 | 名称 | 校验内容 | 校验失败处理 |
|------|------|----------|-------------|
| 1 | 缓冲区抽干 (Anti-Buffer-Bloat) | 收到一帧后，切换非阻塞模式，循环读空缓冲区，只保留最新一帧 | 丢弃旧帧，保证零延迟 |
| 2 | 长度 + Magic Number | 包长必须为 29 字节，Magic 必须为 `0x55AA` | 丢弃非法包 |
| 3 | CRC16-CCITT | 对前 27 字节计算 CRC16，比对包尾 CRC 字段 | 丢弃损坏数据包 |
| 4 | 序列号管理 | 丢弃 `seq <= last_seq` 的旧包/重复包 | 丢弃乱序包，防时空倒流 |
| 5 | 断连重连 | 超过 `reconnect_timeout_sec` 未收到合法数据 | 自动重建 Socket |

### CRC16 实现

- 算法：CCITT-CRC16
- 多项式：`0x1021`
- 初始值：`0xFFFF`（与 Jetson 端 STM32 HAL_CRC 一致）
- 覆盖范围：包的前 27 字节（不含 CRC 字段本身）
- 实现方式：编译期 constexpr 查表法，运行时零开销

---

## 附录 A：永久网络配置

如果你不想每次重启后手动配 IP，可以使用 Netplan 永久配置。

在 PC 上创建 `/etc/netplan/01-vision.yaml`：

```yaml
network:
  version: 2
  ethernets:
    enp3s0:                        # ← 改成你实际的网口名
      dhcp4: no
      addresses:
        - 192.168.10.100/24
```

应用配置：

```bash
sudo netplan apply
```

在 Jetson 上同样操作，IP 改为 `192.168.10.101/24`。

---

## 附录 B：test_sender.py 说明

`test_sender.py` 是独立的 Python 脚本，**不依赖 ROS 2**，可以在任何有 Python 3 的机器上运行。

功能：
- 按照 29 字节协议构造 UDP 包
- 自动计算正确的 CRC16 校验值
- 模拟正弦运动轨迹的假目标
- 支持自定义发包频率和目标数量

**修改区（只需改第 21-25 行）：**

```python
SEND_TO_IP   = "192.168.2.100"   # PC 的网口 IP
SEND_TO_PORT = 12345              # 与 vision_bridge_node 的 udp_port 一致
SEND_RATE_HZ = 30                # 发包频率（Hz）
TARGET_COUNT = 1                 # 模拟目标数量（每个 track_id 独立）
```

---

## 附录 C：完整工作流速查

```
# ===== PC 端（接收） =====
# 1. 配网口
sudo ip addr add 192.168.10.100/24 dev enp3s0
sudo ip link set enp3s0 up

# 2. 编译
cd ~/GMK && colcon build --packages-select techx_vision_bridge

# 3. 启动
source install/setup.bash
ros2 launch techx_vision_bridge vision_bridge.launch.py

# 4. 验证（另开终端）
source install/setup.bash
ros2 topic echo /techx/vision/targets


# ===== Jetson 端（发送） =====
# 1. 配网口
sudo ip addr add 192.168.10.101/24 dev eth0
sudo ip link set eth0 up

# 2. 修改 config.json 中的 target_ip 为 "192.168.10.100"

# 3. 启动 TECHX_vision
cd TECHX_vision && ./start_jetson.sh

# 或者用模拟脚本
# 修改 test_sender.py 中的 SEND_TO_IP = "192.168.10.100"
# python3 test_sender.py
```