# GMK 折线式视觉伺服

该包提供 `/weapon_visual_servo` Action。执行时订阅 `/weapon_target`，先仅沿机器人
Y 轴对准目标像素，再仅沿 X 轴对准目标距离，最终向 `/cmd_vel` 发布速度。

## 构建与启动

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select gmk_visual_servo_interfaces gmk_weapon_bridge gmk_visual_servo
source install/setup.bash
ros2 launch gmk_visual_servo weapon_visual_servo.launch.py
```

组合 launch 会同时启动 UDP bridge 和视觉伺服 Action Server；不要另外重复启动
`weapon_bridge.launch.py`。

## 发起请求

下面的请求表示最终让相机与目标保持 0.5 m，整个动作最多执行 10 s：

```bash
ros2 action send_goal /weapon_visual_servo \
  gmk_visual_servo_interfaces/action/VisualServo \
  "{target_distance_m: 0.5, timeout_sec: 10.0}" \
  --feedback
```

`target_distance_m` 是相机到目标的期望距离，不是当前测量值。参数与限速位于
`config/visual_servo.yaml`。

`min_vx_mps/min_vy_mps` 是底盘克服静摩擦所需的最小非零速度。当控制器仍需纠偏、
但按增益计算出的速度更小时，会保留方向并提升到对应最小值；进入容差或停车状态时
仍然输出严格的零速度。把最小值设为 `0.0` 可以关闭该轴的最小速度限制。

## 安全约束

- Action 执行期间不得有其他节点同时发布 `/cmd_vel`。
- 第一次实车测试前应架空底盘或进一步降低 `max_vx_mps/max_vy_mps`。
- 目标无效、视觉数据超时、取消、失败和节点退出都会立即发布零速度。
