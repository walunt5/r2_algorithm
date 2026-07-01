# r2_operator_console

R2 综合遥控与动作客户端 UI。

该包只启动客户端，不启动或修改现有导航、机械臂、底盘串口节点。运行前应先启动导航系统、`techx_r2_arm_control` 和 `techx_r2_chassis_control`。

## 启动

```bash
ros2 launch r2_operator_console operator_console.launch.py
```

可覆盖参数示例：

```bash
ros2 launch r2_operator_console operator_console.launch.py \
  goals_file:=/home/xie/techx_R2_algorithm/r2_algorithm/install/r2_nav_bringup/share/r2_nav_bringup/config/r2_nav_goals.yaml \
  default_vx:=0.30 default_vy:=0.30 default_wz:=0.60
```

## 功能

- `/cmd_vel` 手动遥控，保留 `teleop_twist_keyboard` 键位和 Shift 全向键位。
- 按住式全向移动按钮，20 Hz 发布；松开、失焦、关闭、导航开始、升降开始时发布零速度。
- `/r2_arm/execute_action` 机械臂姿态和夹爪开合。
- `/r2_chassis/lift_control` 整体升降，固定 `mask=7`。
- `/r2_navigate_to_pose` 预设点导航与取消。
- 手动、导航、升降互斥；机械臂独立执行但不允许机械臂动作并发。
