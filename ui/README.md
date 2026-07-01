# R2 Touch UI

PyQt5 触控界面，固定分辨率为 `900x500`。界面负责选择红/蓝方、加载一区或三区地图、启动总系统，以及启动一区、二区和三区行为树。

## 运行

```bash
cd /home/xie/techx_R2_algorithm/r2_algorithm/ui
pip3 install -r requirements.txt
./run.sh
```

也可以指定工程根目录：

```bash
export R2_ALGORITHM_ROOT=/home/xie/techx_R2_algorithm/r2_algorithm
python3 main.py
```

## 系统启动模式

1. 先选择红方或蓝方。
2. 点击“启动一区系统”或“启动三区系统”。
3. UI 根据队伍和区域读取 `config/field_profiles.yaml`。
4. UI 将地图路径写入源码和 `install/share` 中的 `r2_nav_params.yaml`，并同步所选队伍的导航目标点文件。
5. 两种模式都启动：

   ```bash
   ros2 launch r2_bt_bringup r2_task_mock_bringup.launch.py
   ```

6. 检测到 `map -> chassis_base_link` 后，系统进入 READY。

入口互斥规则：

- 一区系统 READY：允许进入一区和二区任务，禁用三区任务。
- 三区系统 READY：只允许进入三区任务。
- 切换队伍或区域前必须先复位当前系统。

## 地图档案

`config/field_profiles.yaml` 按“队伍 -> 区域”组织。红、蓝方各自共用一个导航目标点文件，不同区域可以把目标点继续保存在同一个文件中。

```yaml
red:
  nav_goals_file: "/path/to/red_r2_nav_goals.yaml"
  zone1:
    relocalization_bin_file: "/path/to/zone1.bin"
    relocalization_pcd_file: "/path/to/zone1.pcd"
    map_package_dir: "/path/to/zone1_map_package"
  zone3:
    relocalization_bin_file: "/path/to/zone3.bin"
    relocalization_pcd_file: "/path/to/zone3.pcd"
    map_package_dir: "/path/to/zone3_map_package"
```

正式地图目录固定为：

- `chassis_maps/red_zone1`
- `chassis_maps/blue_zone1`
- `chassis_maps/red_zone3`
- `chassis_maps/blue_zone3`

每个目录都包含重定位 BIN、可视化 PCD 和 OctoMap 地图包。后续替换地图时，修改对应目录内容并同步更新 `field_profiles.yaml` 中的文件名即可。

## 行为树入口

- 一区：`zone1_competition_task.xml`
- 二区：`zone2_competition_task.xml`
- 三区：`zone3_competition_task.xml`

三个正式行为树统一放在：

```text
src/r2_bt_executor/config/zone1_competition_task.xml
src/r2_bt_executor/config/zone2_competition_task.xml
src/r2_bt_executor/config/zone3_competition_task.xml
```

UI 只启动这三个正式文件，目录中的其他 XML 均作为开发和单项测试文件。一区、二区正式文件当前分别复制自原有 `gym_task.xml` 和 `meilin_zone2_task.xml`；三区正式文件是安全等待占位树，不执行机器人动作。

替换任一正式行为树后执行：

```bash
cd /home/xie/techx_R2_algorithm/r2_algorithm
colcon build --symlink-install --packages-select r2_bt_executor r2_bt_bringup
source install/setup.bash
```

## 二区路线和 KFS

二区页面继续编辑 `src/r2_bt_executor/config/meilin_map.yaml`：

- 路线模式用于选择方块顺序。
- KFS 模式用于切换 `blocks.Bx.has_kfs`。
- 保存时同步源码和 `install/share` 配置。
