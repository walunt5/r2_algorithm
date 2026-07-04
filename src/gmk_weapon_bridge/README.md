# GMK 武器头视觉桥接包

这是 GMK 端 ROS2 功能包。作用只有一个：接收 Jetson 发来的快接头识别 UDP 包，并发布 `/weapon_target`。

## 放到总工程

把整个目录放进总工程的 `src` 下，例如：

```text
gmk_ws/
├── src/
│   └── gmk_weapon_bridge/
│       ├── package.xml
│       ├── setup.py
│       ├── config/
│       ├── launch/
│       └── gmk_weapon_bridge/
└── ...
```

然后在总工程根目录：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select gmk_weapon_bridge
source install/setup.bash
```

## 运行

推荐用 launch，参数来自 `config/weapon_bridge.yaml`：

```bash
ros2 launch gmk_weapon_bridge weapon_bridge.launch.py
```

也可以直接运行节点：

```bash
ros2 run gmk_weapon_bridge bridge_node
```

覆盖参数：

```bash
ros2 run gmk_weapon_bridge bridge_node --ros-args \
  -p udp_bind:=0.0.0.0 \
  -p udp_port:=12345 \
  -p topic:=/weapon_target
```

## 发布话题

`/weapon_target`，类型 `std_msgs/msg/Float32MultiArray`：

```text
data = [valid, u, v, z_m, conf]
valid: 1 有目标 / 0 无目标
u, v : 快接头像素中心，给底盘左右对准
z_m  : 深度，单位米，给前后距离判断
conf : YOLO 置信度
```

## 自测

GMK 单机全链路测试，三个终端：

```bash
ros2 launch gmk_weapon_bridge weapon_bridge.launch.py
ros2 run gmk_weapon_bridge mock_jetson_sender
ros2 topic echo /weapon_target --qos-reliability best_effort
```

能看到 `u/v/z/conf` 周期变化，说明 GMK 端“UDP 收包 -> 解码 -> ROS2 话题”正常。

桥接节点按视觉控制的低延迟语义发布：`BEST_EFFORT + KEEP_LAST(1)`，只保留最新帧。
命令行观察时显式使用匹配的 QoS：

```bash
ros2 topic echo /weapon_target --qos-reliability best_effort
ros2 topic hz /weapon_target --qos-reliability best_effort
```

节点每 5 秒输出一次统计：`rx` 是 UDP 有效包速率，`pub` 是 ROS 发布速率，
`age` 是最后一包到达 GMK 后经过的时间，`superseded` 是为避免旧帧积压而主动跳过的包数。
当前协议没有 Jetson 发送时间戳，因此 `age` 不是 Jetson 到 GMK 的端到端延迟；如需测量真实端到端延迟，协议必须增加发送时间戳并先同步两端时钟。

### 底盘控制端约束

视觉数据用于闭环控制时，控制节点必须：

- 使用 `BEST_EFFORT + KEEP_LAST(1)` 订阅 `/weapon_target`，不能缓存历史帧；
- 收到 `valid=0` 时立即下发零速度；
- 使用单调时钟记录最后一次回调时间，超过 `100 ms`（30 FPS 下约 3 帧）未收到新帧时立即下发零速度；
- 不得在丢包、断网或节点退出后继续沿用上一帧目标。

接真 Jetson 后，如果 `/weapon_target` 没数据，先停掉 bridge，再跑：

```bash
ros2 run gmk_weapon_bridge verify_udp
```

能刷出包：网络和 Jetson 发送正常，问题在 ROS2 节点/编译/source。
刷不出包：优先查 IP、网线、防火墙、Jetson 是否真的在发。

## 协议

与 Jetson `weapon_vision.py` 一致：

```text
<H I B f f f f> 小端，共 23 字节
magic=0x5701, seq, valid, u, v, z_m, conf
```

默认网络：

```text
Jetson: 192.168.10.101
GMK   : 192.168.10.100
UDP   : 12345
```
