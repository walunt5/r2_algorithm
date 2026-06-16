# TECHX_vision — Jetson Orin NX 部署与通信指南

## 一、快速开始（TL;DR）

```bash
# 1. 修改 config.json 里的 IP
vim config.json        # 把 target_ip 改成 GMK 主控的 IP

# 2. 启动
./start_jetson.sh
```

**就这么简单。** 换赛场只需要改 `config.json` 里一行 IP，不需要改代码。

---

## 二、网络拓扑

```
┌─────────────────────┐         UDP 29字节帧          ┌─────────────────────┐
│   Jetson Orin NX    │ ═══════════════════════════════│    GMK 主控          │
│                     │     以太网直连 / 交换机          │                     │
│  IP: 192.168.10.101 │ ──────────────────────────────│  IP: 192.168.10.100  │
│                     │                                │                     │
│  ├─ Orbbec Gemini   │                                │  接收并解析坐标      │
│  │  335L (USB3.2)   │                                │  控制机械臂抓取      │
│  └─ TECHX_vision    │                                │                     │
└─────────────────────┘                                └─────────────────────┘
```

**要求：** Jetson 和 GMK 在同一子网，能互相 ping 通。

---

## 三、config.json 配置文件

项目根目录下的 `config.json` 是唯一需要修改的配置文件：

```json
{
    "udp": {
        "target_ip": "192.168.10.100",
        "target_port": 12345
    }
}
```

| 字段 | 说明 | 示例 |
|------|------|------|
| `target_ip` | GMK 主控的 IP 地址 | `"192.168.1.100"` |
| `target_port` | UDP 通信端口 | `12345` |

**换赛场时只改 `target_ip` 即可，端口一般不用动。**

---

## 四、环境准备（首次部署）

### 4.1 Python 环境

```bash
# 安装 Miniconda（ARM64 版本）
wget https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-aarch64.sh
bash Miniforge3-Linux-aarch64.sh
# 重启终端

# 创建环境
conda create -n techx python=3.10 -y
conda activate techx
```

### 4.2 安装依赖

```bash
conda activate techx

# 核心依赖
pip install ultralytics opencv-python numpy Pillow

# 可选：ONNX 加速（CPU 推理时推荐）
pip install onnxruntime

# 可选：串口通信
pip install pyserial
```

### 4.3 拷贝项目

```bash
scp -r TECHX_vision/ user@192.168.10.101:/home/user/
```

### 4.4 验证

```bash
cd TECHX_vision
python3 -c "from ultralytics import YOLO; print('OK')"
python3 -c "from udp_streamer import VisionUdpStreamer; print('OK')"
```

---

## 五、UDP 通信协议

### 5.1 帧格式（29 字节，小端序）

```
┌────────┬──────────┬───────────┬──────────┬──────────────────────┬───────┐
│ Magic  │ Sequence │ Timestamp │ Track ID │  Xc    Yc    Zc      │ CRC16 │
│ uint16 │  uint32  │  float64  │  uint8   │ f32    f32    f32    │uint16 │
│ 0x55AA │  自增    │   秒      │  追踪ID  │  米    米    米      │ 校验  │
│ [0:2]  │  [2:6]   │  [6:14]   │  [14]    │ [15:27]             │[27:29]│
└────────┴──────────┴───────────┴──────────┴──────────────────────┴───────┘
```

| 偏移 | 字节 | 类型 | 含义 |
|------|------|------|------|
| 0 | 2 | uint16 | 帧头 `0x55AA` |
| 2 | 4 | uint32 | 序列号（自增） |
| 6 | 8 | float64 | 硬件时间戳（秒） |
| 14 | 1 | uint8 | 追踪 ID（`tid & 0xFF`） |
| 15 | 4 | float32 | Xc（相机坐标 X，米） |
| 19 | 4 | float32 | Yc（相机坐标 Y，米） |
| 23 | 4 | float32 | Zc（相机坐标 Z，米，深度） |
| 27 | 2 | uint16 | CRC16-CCITT 校验 |

**Python struct 格式：** `'<H I d B 3f H'`

### 5.2 坐标含义

```
相机坐标系（右手系）：
    Xc → 右（水平）
    Yc → 下（垂直）
    Zc → 前（深度方向）

单位：米
原点：相机光心
```

### 5.3 发送行为

| 特性 | 说明 |
|------|------|
| 发送条件 | 检测到目标 且 深度有效（Zc > 0） |
| 发送频率 | 每帧都发（~15-30 Hz） |
| 发送内容 | 最近目标（深度最小者优先） |
| 失败处理 | 非阻塞，网线拔出不崩溃不卡顿 |

### 5.4 CRC16-CCITT 校验

```
多项式: 0x1021
初始值: 0xFFFF
校验范围: 前 27 字节
与 STM32 HAL_CRC 一致
```

**GMK 端 C 语言校验代码：**

```c
uint16_t crc16_ccitt(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; j++)
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
    }
    return crc;
}
```

---

## 六、启动方式

```bash
cd TECHX_vision
./start_jetson.sh
```

启动脚本会自动：检查 conda 环境 → 检查依赖 → 检查相机 → 检查 CUDA → 启动程序。

也可以用 Python 直接启动：

```bash
conda activate techx
python kfs_ui/main.py
```

---

## 七、UI 操作

| 操作 | 方式 |
|------|------|
| 切换模型 | 右侧面板【0】Combobox |
| 切换推理后端 | 右侧面板【1】PyTorch / ONNX |
| 调节阈值 | 右侧面板【2】滑块 |
| 查看 UDP 状态 | 右侧面板【6】 |
| 暂停/继续 | 空格键 |
| 深度热力图 | D 键 |
| 退出 | Q 键 |

---

## 八、排查通信问题

```bash
# 1. 确认网络连通
ping 192.168.10.100

# 2. 在 GMK 端抓包验证
sudo tcpdump -i eth0 udp port 12345 -X

# 3. 在 Jetson 端发送测试帧
python3 -c "
from udp_streamer import VisionUdpStreamer
import time
s = VisionUdpStreamer('192.168.10.100', 12345)
s.send_target(timestamp=time.time(), tid=1, x=0.5, y=0.3, z=1.2)
print('Test frame sent')
"

# 4. 检查防火墙
sudo ufw status
```

---

## 九、GMK 端 Python 接收示例

```python
"""GMK 端 UDP 接收"""
import socket, struct

FRAME_FMT = '<H I d B 3f H'

def crc16_ccitt(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = (crc << 1) ^ 0x1021 if crc & 0x8000 else crc << 1
            crc &= 0xFFFF
    return crc

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", 12345))

while True:
    data, addr = sock.recvfrom(64)
    if len(data) != 29:
        continue
    if crc16_ccitt(data[:27]) != struct.unpack_from('<H', data, 27)[0]:
        continue
    magic, seq, ts, tid, xc, yc, zc, _ = struct.unpack(FRAME_FMT, data)
    print(f"[#{seq}] Track{tid}: Xc={xc:.4f} Yc={yc:.4f} Zc={zc:.4f}")
```

---

## 十、检测类别

| 模型 | 类别 | 置信度阈值 |
|------|------|-----------|
| KFS 目标检测 | `fake_kfs` | 0.15 |
| | `r1_kfs_red` / `r1_kfs_blue` | 0.25 |
| | `r2_kfs_red` / `r2_kfs_blue` | 0.25 |
| 手势识别 | `fist` / `palm` / `sprea` | 0.4~0.5 |

---

## 十一、全部可调参数速查表

> 以下列出项目中所有可修改的参数，含精确的文件路径和行号。
> 编号格式 `L123` 表示该参数定义在第 123 行附近。

### 11.1 通信配置

| 参数 | 文件 | 位置 | 说明 |
|------|------|------|------|
| UDP 目标 IP | `config.json` | `target_ip` | 改成 GMK 主控的 IP |
| UDP 目标端口 | `config.json` | `target_port` | 默认 `12345` |
| UDP 魔数 | `udp_streamer.py` | L34 `MAGIC` | 帧头 `0x55AA`，与 GMK 端一致即可 |
| 串口默认端口 | `kfs_ui/main.py` | L198 | 运行时 UI 面板可改 |

### 11.2 模型路径

| 参数 | 文件 | 位置 | 说明 |
|------|------|------|------|
| KFS 模型 .pt | `kfs_ui/main.py` | L34 | 换模型时改路径 |
| KFS 模型 .onnx | `kfs_ui/main.py` | L35 | 换模型时改路径 |
| 手势模型 .pt | `kfs_ui/main.py` | L37 | 换模型时改路径 |
| 手势模型 .onnx | `kfs_ui/main.py` | L38 | 换模型时改路径 |

### 11.3 检测阈值

| 参数 | 文件 | 位置 | 默认值 | 说明 |
|------|------|------|--------|------|
| 各类别置信度阈值 | `kfs_ui/main.py` | L40-44 `CLASS_CONFS` | 见上表 | 每个类别独立阈值，降低=更敏感但误检多 |
| 全局置信度 slider | `kfs_ui/main.py` | L75 | `0.25` | 运行时 UI 滑块可调 |
| 全局 IoU slider | `kfs_ui/main.py` | L76 | `0.45` | 运行时 UI 滑块可调 |

### 11.4 几何过滤

| 参数 | 文件 | 位置 | 默认值 | 说明 |
|------|------|------|--------|------|
| 最小框面积 | `kfs_ui/detector.py` | L119 `bw * bh < 500` | 500 px | 小于此面积丢弃 |
| 最大宽高比 | `kfs_ui/detector.py` | L119 `bw > bh * 4` | 4:1 | 超过此比例丢弃（排除长条噪点） |
| 边缘安全距离 | `kfs_ui/detector.py` | L121 | cx<32 / cx>608 / cy<24 / cy>456 | 目标中心触碰画面边缘时丢弃 |

### 11.5 追踪参数

| 参数 | 文件 | 位置 | 默认值 | 说明 |
|------|------|------|--------|------|
| IOU 匹配阈值 | `kfs_ui/detector.py` | L142 `best_i = 0.3` | 0.3 | IOU 大于此值才匹配 |
| 中心距离兜底 | `kfs_ui/detector.py` | L153 `dist < 80` | 80 px | 快速移动时 IOU 很低，用中心距离补救 |
| 跟踪确认帧数 | `kfs_ui/pipeline.py` | L111 `confirm_count < 3` | 3 帧 | 新目标连续出现 N 帧后才显示 |
| 跟踪过期帧数 | `kfs_ui/detector.py` | L164 `last_seen < 30` | 30 帧 | 超过 N 帧未匹配则删除 track |
| IOU 合并阈值 | `kfs_ui/detector.py` | L130 `> 0.3` | 0.3 | 同类别框重叠超过此值合并 |

### 11.6 深度测距

| 参数 | 文件 | 位置 | 默认值 | 说明 |
|------|------|------|--------|------|
| 深度标定偏移 | `kfs_ui/pipeline.py` | L7 `DEPTH_OFFSET_MM` | -50 mm | 负值=传感器读数偏大，用卷尺实测拟合 |
| 深度标定缩放 | `kfs_ui/pipeline.py` | L8 `DEPTH_SCALE_CORR` | 1.0 | 线性缩放修正 |
| 有效深度范围 | `kfs_ui/detector.py` | L70 `(200, 8000)` | 200~8000 mm | 超出此范围的深度值丢弃 |
| ROI 渐扩比例 | `kfs_ui/detector.py` | L64 `[0.3, 0.5, 0.7]` | 30%/50%/70% | 逐级扩大采样区域 |
| 最小有效像素 | `kfs_ui/detector.py` | L71 `len(v) >= 5` | 5 | ROI 内至少 N 个有效深度像素 |
| MAD 飞点阈值 | `kfs_ui/detector.py` | L73 `abs(v - median) < 300` | 300 mm | 偏离中值超过此值视为飞点 |
| 深度平滑系数 | `kfs_ui/detector.py` | L95 `0.6 / 0.4` | 0.6×new + 0.4×history | 新值权重越大响应越快但越抖 |
| 深度衰减系数 | `kfs_ui/detector.py` | L91 `prev * 0.85` | 0.85 | 遮挡时深度值每帧衰减率 |

### 11.7 颜色分类

| 参数 | 文件 | 位置 | 默认值 | 说明 |
|------|------|------|--------|------|
| 红色 HSV 范围 | `kfs_ui/detector.py` | L50 | `(0,50,50)~(10,255,255)` + `(160,50,50)~(180,255,255)` | 两个区间覆盖红色色相环 |
| 蓝色 HSV 范围 | `kfs_ui/detector.py` | L51 | `(100,50,50)~(130,255,255)` | |
| 最小彩色像素 | `kfs_ui/detector.py` | L53 `r + b < 50` | 50 | 低于此数判为不确定 |
| 判定倍率 | `kfs_ui/detector.py` | L54 `r > b * 1.5` | 1.5 | 一方像素数 > 另一方 × N 才判定 |
| 颜色投票帧数 | `kfs_ui/pipeline.py` | L123 `hist[-5:]` | 5 帧 | 取最近 N 帧颜色的众数 |

### 11.8 相机参数

| 参数 | 文件 | 位置 | 默认值 | 说明 |
|------|------|------|--------|------|
| 相机内参 fx, fy, cx, cy | `kfs_ui/main.py` | L83 | 367.93, 368.05, 318.75, 236.20 | Orbbec 模式下自动从相机读取并覆盖 |
| 相机分辨率 | `kfs_ui/camera.py` | L69-71 | 640×480@30fps | 彩色+深度流配置 |
| UVC 扫描范围 | `kfs_ui/camera.py` | L93 | index 0~4 | 扫描 USB 摄像头时的索引范围 |

### 11.9 主循环

| 参数 | 文件 | 位置 | 默认值 | 说明 |
|------|------|------|--------|------|
| 窗口大小 | `kfs_ui/main.py` | L64 | `"1180x680"` | Tkinter 窗口尺寸 |
| 主循环周期 | `kfs_ui/main.py` | L333 `after(33)` | 33 ms | ~30 FPS 刷新率 |

---

## 修改示例

**场景 1：换赛场，GMK 主控 IP 变了**

> 改 `config.json` → `target_ip`

**场景 2：误检太多，想提高检测门槛**

> 改 `kfs_ui/main.py` L40-44 `CLASS_CONFS`，把对应类别的阈值加大（如 0.25 → 0.4）

**场景 3：目标快速移动时 track 容易断**

> 改 `kfs_ui/detector.py` L153，把中心距离兜底从 `80` 加大到 `120`

**场景 4：深度测距偏大/偏小，需要标定**

> 改 `kfs_ui/pipeline.py` L7 `DEPTH_OFFSET_MM`，正值=加补偿，负值=减补偿

**场景 5：换了一个新的 YOLO 模型**

> 改 `kfs_ui/main.py` L32-38，把 `.pt` 和 `.onnx` 路径指向新模型，同时更新 L40-44 `CLASS_CONFS` 为新模型的类别名和阈值

**场景 6：颜色识别不准，红蓝经常跳变**

> 改 `kfs_ui/detector.py` L50-51 的 HSV 范围，或改 `kfs_ui/pipeline.py` L123 把投票帧数从 5 加大到 10

**场景 7：深度值偶尔突变（飞点没滤干净）**

> 改 `kfs_ui/detector.py` L73，把 MAD 阈值从 `300` 减小到 `200`（更严格过滤），或改 L95 平滑系数让新值权重更小（如 `0.4/0.6`）