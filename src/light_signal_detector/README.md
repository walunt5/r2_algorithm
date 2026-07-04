# light_signal_detector：高亮橙色灯带信号检测

`light_signal_detector` 是一个独立的 ROS 2 功能包。它从普通 RGB 相机图像中识别大面积高亮橙色灯带，发布稳定的 ON/OFF 信号，并提供等待新 ON 信号的 Action 服务。

它不依赖 YOLO、深度图、Gemini 相机 SDK、`weapon_perception` 或 `camera_visual_servo`，因此可以单独构建和运行，也可以更换为其他 RGB 相机。

## 1. 工作原理

节点订阅一幅 RGB 图像后依次执行：

1. 将 BGR 图像转换为 OpenCV HSV。
2. 提取高亮橙色区域。
3. 提取与橙色光晕接触的暖白色过曝核心。
4. 使用形态学闭运算连接灯带中的小间隙。
5. 选取最大的连通区域。
6. 同时检查候选面积、画面占比、过曝核心像素数和核心占比。
7. 连续多帧满足条件后输出 ON，连续多帧失败后输出 OFF。

最终判定条件为：

```text
候选面积 >= max(min_component_area,
                  图像宽度 × 图像高度 × min_component_area_ratio)
并且
过曝核心像素数 >= min_white_core_pixels
并且
过曝核心面积 / 候选面积 >= min_white_core_ratio
```

这比单纯判断 HSV 更可靠：普通黄色或橙色物体即使颜色接近，也通常没有足够大的过曝核心。

## 2. ROS 接口

### 订阅

| 话题 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| 彩色图 | `sensor_msgs/msg/Image` | `/camera/color/image_raw` | 必须能由 `cv_bridge` 转为 `bgr8` |

当前版本不直接订阅 `sensor_msgs/msg/CompressedImage`。如果相机只发布压缩图，需要先使用 ROS 图像传输工具解压成 `sensor_msgs/msg/Image`。

### 发布

| 话题 | 类型 | 说明 |
|---|---|---|
| `/light_signal/on` | `std_msgs/msg/Bool` | `true` 表示灯带信号确认成功，否则为 `false` |
| `/light_signal/debug_image` | `sensor_msgs/msg/Image` | 带检测掩膜、轮廓和判定数据的调试图 |

如果超过 `image_timeout_sec` 没有收到图像，节点会持续发布 `false`，避免下游保留过期的 ON 状态。

### Action

| 名称 | 类型 | 说明 |
|---|---|---|
| `/r2_light_signal/wait` | `r2_light_interfaces/action/WaitForLightSignal` | 在指定时间内等待 Goal 开始后发布的新 `true` |

等待期间的新 `false` 只通过 feedback 报告，不会提前结束 Action。收到新 `true` 时成功；超时仍未收到新 `true` 时失败。Goal 开始前残留的旧 `true` 不算本次成功。

## 3. 环境要求

- ROS 2 Humble 或兼容版本
- `rclpy`
- `sensor_msgs`、`std_msgs`
- `cv_bridge`
- Python 3、NumPy、OpenCV
- 一个发布 `sensor_msgs/msg/Image` 的 RGB 相机节点

Ubuntu/ROS 二进制依赖通常可以使用以下命令安装：

```bash
sudo apt update
sudo apt install \
  ros-$ROS_DISTRO-cv-bridge \
  ros-$ROS_DISTRO-rqt-image-view \
  python3-numpy \
  python3-opencv
```

## 4. 构建

进入包含 `src/` 的工作区根目录：

```bash
cd /home/xie/techx_R2_algorithm/r2_algorithm
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to light_signal_detector
source install/setup.bash
```

确认包和可执行文件已安装：

```bash
ros2 pkg prefix light_signal_detector
ros2 pkg executables light_signal_detector
```

第二条命令应包含：

```text
light_signal_detector light_signal_detector_node.py
light_signal_detector light_signal_wait_action_server.py
```

## 4.1 行为树用法

`r2_bt_executor` 中注册的节点名为 `R2WaitForLightSignalActionNode`：

```xml
<R2WaitForLightSignalActionNode
  action_name="/r2_light_signal/wait"
  timeout_sec="3.0"
  server_timeout_ms="3000"
  result_grace_ms="2000" />
```

节点等待期间返回 `RUNNING`，Action 成功映射为 `SUCCESS`，Action 超时、拒绝、取消异常或服务不可用映射为 `FAILURE`。完整示例位于 `r2_bt_executor/config/test_light_signal_action.xml`。

## 5. 启动相机并确认图像

先启动你的 RGB 相机。以 Gemini 335 为例：

```bash
ros2 launch orbbec_camera gemini_330_series.launch.py
```

查找图像话题：

```bash
ros2 topic list | grep -E "image_raw|color|rgb"
```

确认消息类型和帧率：

```bash
ros2 topic type /camera/color/image_raw
ros2 topic hz /camera/color/image_raw
```

消息类型应为：

```text
sensor_msgs/msg/Image
```

## 6. 启动检测节点

使用默认彩色图话题：

```bash
ros2 launch light_signal_detector light_signal.launch.py
```

如果相机话题不同，在启动时指定：

```bash
ros2 launch light_signal_detector light_signal.launch.py \
  color_topic:=/你的相机/rgb/image_raw
```

也可以直接运行节点并加载配置：

```bash
ros2 run light_signal_detector light_signal_detector_node.py \
  --ros-args \
  --params-file src/light_signal_detector/config/light_signal.yaml
```

## 7. 查看检测结果

查看 ON/OFF 信号：

```bash
ros2 topic echo /light_signal/on
```

输出示例：

```yaml
data: true
---
```

查看调试图：

```bash
ros2 run rqt_image_view rqt_image_view /light_signal/debug_image
```

调试图颜色含义：

- 橙色覆盖：HSV 高亮橙色掩膜。
- 白色覆盖：与橙色区域相连的过曝核心。
- 绿色轮廓和矩形：当前最大的候选连通区域。
- `SIGNAL`：经过连续帧确认后的最终 ON/OFF 状态。
- `frame`：当前单帧是否满足全部条件。
- `area=实际值/要求值`：候选面积与当前分辨率下的最小要求。
- `core=实际值/要求值`：过曝核心像素数。
- `core_ratio=实际值/要求值`：过曝核心占候选面积的比例。

`frame=detected` 连续达到 `on_frames` 后，`SIGNAL` 才会变为 ON。

## 8. 参数说明

默认配置文件为：

```text
src/light_signal_detector/config/light_signal.yaml
```

| 参数 | 默认值 | 作用 |
|---|---:|---|
| `color_topic` | `/camera/color/image_raw` | 输入 RGB 图像话题 |
| `signal_topic` | `/light_signal/on` | ON/OFF 输出话题 |
| `debug_image_topic` | `/light_signal/debug_image` | 调试图输出话题 |
| `publish_debug_image` | `true` | 是否生成调试图 |
| `h_min` | `5` | 橙色色相下限，OpenCV 范围 0～179 |
| `h_max` | `22` | 橙色色相上限；过大会包含黄色 |
| `s_min` | `80` | 橙色区域最小饱和度 |
| `v_min` | `210` | 橙色区域最小亮度 |
| `white_s_max` | `160` | 暖白/黄白过曝核心最大饱和度 |
| `white_v_min` | `245` | 过曝核心最小亮度 |
| `white_adjacency_kernel_size` | `9` | 判断核心是否接触橙色光晕的膨胀核尺寸，必须为正奇数 |
| `close_kernel_size` | `7` | 连接掩膜间隙的闭运算核尺寸，必须为正奇数 |
| `close_iterations` | `2` | 闭运算次数 |
| `min_component_area` | `600.0` | 候选区域最小绝对像素面积 |
| `min_component_area_ratio` | `0.01` | 候选区域至少占整幅图像的比例，0.01 表示 1% |
| `min_white_core_pixels` | `50` | 过曝核心最小像素数 |
| `min_white_core_ratio` | `0.05` | 过曝核心至少占候选区域的比例，0.05 表示 5% |
| `on_frames` | `3` | 连续多少帧成功后输出 ON |
| `off_frames` | `5` | 连续多少帧失败后输出 OFF |
| `image_timeout_sec` | `0.5` | 相机断流后强制 OFF 的超时时间；小于等于 0 表示关闭超时 |

节点每帧读取这些参数，因此可以运行时调节。例如把最小面积提高到画面的 2%：

```bash
ros2 param set /light_signal_detector min_component_area_ratio 0.02
```

## 9. 推荐调参流程

1. 固定相机位置，尽量关闭自动曝光和自动白平衡。
2. 先保持默认参数，打开 `/light_signal/debug_image`。
3. 灯灭时观察普通黄色物体，确认 `frame=not detected`。
4. 灯亮时观察 `area`、`core` 和 `core_ratio`。
5. 每次只调整一类参数，再同时测试灯亮和灯灭场景。

### 灯带亮起却识别不到

- `area` 不足：降低 `min_component_area_ratio`，例如从 `0.01` 调到 `0.005`。
- 没有橙色覆盖：根据实际相机适当修改 `h_min/h_max`，或降低 `v_min`。
- `core` 太少：适当提高 `white_s_max` 或降低 `white_v_min`。
- `core_ratio` 不足：确认整块过曝核心是否显示为白色，再小幅降低 `min_white_core_ratio`。

运行时示例：

```bash
ros2 param set /light_signal_detector white_s_max 180
ros2 param set /light_signal_detector min_white_core_ratio 0.03
```

### 普通黄色物体被误识别

- 降低 `h_max`，例如从 22 调到 20 或 18。
- 提高 `v_min`，让算法只接受更亮的自发光区域。
- 提高 `min_component_area_ratio`，过滤小面积色块。
- 提高 `min_white_core_pixels` 或 `min_white_core_ratio`，要求更明显的过曝核心。

运行时示例：

```bash
ros2 param set /light_signal_detector h_max 20
ros2 param set /light_signal_detector min_component_area_ratio 0.02
```

如果普通物体和灯带在颜色、亮度、面积、过曝核心方面都相似，单靠当前视觉特征无法可靠区分，需要增加固定 ROI、S 形轮廓约束、闪烁编码或物理定位标记。

## 10. 更换相机

更换相机不需要修改代码，只要新相机发布 `sensor_msgs/msg/Image`：

```bash
ros2 launch light_signal_detector light_signal.launch.py \
  color_topic:=/new_camera/image_raw
```

不同相机的曝光、白平衡和颜色响应不同，更换后必须重新检查 HSV 阈值和过曝核心阈值。面积比例参数会自动适配分辨率，但固定像素参数 `min_component_area` 和 `min_white_core_pixels` 仍需根据成像距离检查。

## 11. 测试

运行本包单元测试：

```bash
colcon test --packages-select light_signal_detector
colcon test-result --verbose
```

测试覆盖高亮橙色灯带、暖色过曝核心、普通白色、高亮黄色、暗橙色、小面积反光、面积比例和连续帧去抖。

## 12. 常见问题

### 启动后一直输出 OFF

先检查是否收到图像：

```bash
ros2 topic hz /camera/color/image_raw
ros2 node info /light_signal_detector
```

然后查看调试图中的 `frame`、`area`、`core` 和 `core_ratio`，不要盲目同时放宽全部参数。

### 日志提示 `No color image ... forcing light signal OFF`

说明 `color_topic` 配错、相机没有运行、消息类型不兼容，或者 ROS 2 节点之间无法通信。

### 能看到图像但颜色明显不对

确认相机消息使用标准 ROS 图像编码。节点请求 `bgr8`，由 `cv_bridge` 完成转换。非标准编码需要在相机节点侧转换。

### 修改 YAML 后没有变化

如果使用 `--symlink-install`，通常重启节点即可。否则重新构建并重新 source：

```bash
colcon build --packages-select light_signal_detector
source install/setup.bash
```
