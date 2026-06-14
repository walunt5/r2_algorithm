#!/usr/bin/env python3
import math
import os
import threading
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

import yaml

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.time import Time

import tf2_ros
from ament_index_python.packages import get_package_share_directory


def quaternion_to_yaw(q):
    """
    把 ROS 四元数转换成平面 yaw。
    ROS 四元数顺序是 x, y, z, w。
    """
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class GoalRecorderNode(Node):
    def __init__(self):
        super().__init__("r2_goal_recorder_gui_node")

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer,
            self,
            spin_thread=False,
        )

    def query_pose(self, parent_frame, child_frame):
        """
        查询 child_frame 在 parent_frame 下的位置。
        例如 parent_frame=map, child_frame=chassis_base_link。
        """
        transform = self.tf_buffer.lookup_transform(
            parent_frame,
            child_frame,
            Time(),
        )

        t = transform.transform.translation
        q = transform.transform.rotation
        yaw = quaternion_to_yaw(q)

        return {
            "frame_id": parent_frame,
            "x": float(t.x),
            "y": float(t.y),
            "z": float(t.z),
            "yaw": float(yaw),
        }


class GoalRecorderGui:
    def __init__(self, ros_node):
        self.ros_node = ros_node
        self.current_pose = None
        self.goal_name_to_edge = {}

        self.root = tk.Tk()
        self.root.title("R2 Nav Goals 标定工具")
        self.root.geometry("900x520")

        self.parent_frame_var = tk.StringVar(value="map")
        self.child_frame_var = tk.StringVar(value="chassis_base_link")

        self.map_yaml_path_var = tk.StringVar(value=self.get_default_map_yaml_path())
        self.nav_goals_yaml_path_var = tk.StringVar(value=self.get_default_nav_goals_yaml_path())

        self.goal_name_var = tk.StringVar(value="")
        self.edge_info_var = tk.StringVar(value="未加载点位列表")

        self.x_var = tk.StringVar(value="")
        self.y_var = tk.StringVar(value="")
        self.z_var = tk.StringVar(value="")
        self.yaw_var = tk.StringVar(value="")

        self.status_var = tk.StringVar(value="请先点击“加载 approach 点位列表”")

        self.build_ui()

        # 启动后自动尝试加载一次
        self.root.after(200, self.load_goal_names_from_map_yaml)

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def get_workspace_root(self):
        """
        默认按你的工程路径来找源码文件。
        """
        home = os.path.expanduser("~")
        return os.path.join(
            home,
            "techx_R2_algorithm",
            "r2_algorithm",
        )

    def get_default_map_yaml_path(self):
        source_yaml = os.path.join(
            self.get_workspace_root(),
            "src",
            "r2_bt_executor",
            "config",
            "meilin_map.yaml",
        )

        if os.path.exists(source_yaml):
            return source_yaml

        return ""

    def get_default_nav_goals_yaml_path(self):
        """
        优先保存源码目录下的 r2_nav_goals.yaml。
        注意：如果保存源码 YAML 后，正在运行的节点不会自动重新读取，
        一般需要重新启动相关节点，或者重新 build 后再启动。
        """
        source_yaml = os.path.join(
            self.get_workspace_root(),
            "src",
            "r2_nav_bringup",
            "config",
            "r2_nav_goals.yaml",
        )

        if os.path.exists(source_yaml):
            return source_yaml

        try:
            share_dir = get_package_share_directory("r2_nav_bringup")
            install_yaml = os.path.join(
                share_dir,
                "config",
                "r2_nav_goals.yaml",
            )
            return install_yaml
        except Exception:
            return ""

    def build_ui(self):
        padding = {"padx": 8, "pady": 6}
        row = 0

        tk.Label(self.root, text="父坐标系 parent_frame").grid(
            row=row, column=0, sticky="e", **padding
        )
        tk.Entry(self.root, textvariable=self.parent_frame_var, width=32).grid(
            row=row, column=1, sticky="w", **padding
        )

        tk.Label(self.root, text="子坐标系 child_frame").grid(
            row=row, column=2, sticky="e", **padding
        )
        tk.Entry(self.root, textvariable=self.child_frame_var, width=32).grid(
            row=row, column=3, sticky="w", **padding
        )

        row += 1

        tk.Label(self.root, text="任务地图 meilin_map.yaml").grid(
            row=row, column=0, sticky="e", **padding
        )
        tk.Entry(self.root, textvariable=self.map_yaml_path_var, width=82).grid(
            row=row, column=1, columnspan=3, sticky="w", **padding
        )

        row += 1

        tk.Button(self.root, text="选择 meilin_map.yaml", command=self.select_map_yaml).grid(
            row=row, column=1, sticky="w", **padding
        )
        tk.Button(self.root, text="加载 approach 点位列表", command=self.load_goal_names_from_map_yaml).grid(
            row=row, column=2, sticky="w", **padding
        )

        row += 1

        tk.Label(self.root, text="导航点位 r2_nav_goals.yaml").grid(
            row=row, column=0, sticky="e", **padding
        )
        tk.Entry(self.root, textvariable=self.nav_goals_yaml_path_var, width=82).grid(
            row=row, column=1, columnspan=3, sticky="w", **padding
        )

        row += 1

        tk.Button(self.root, text="选择 r2_nav_goals.yaml", command=self.select_nav_goals_yaml).grid(
            row=row, column=1, sticky="w", **padding
        )

        row += 1

        tk.Label(self.root, text="选择要保存的 approach 点位").grid(
            row=row, column=0, sticky="e", **padding
        )

        self.goal_name_combo = ttk.Combobox(
            self.root,
            textvariable=self.goal_name_var,
            width=42,
            state="readonly",
        )
        self.goal_name_combo.grid(row=row, column=1, sticky="w", **padding)
        self.goal_name_combo.bind("<<ComboboxSelected>>", self.on_goal_selected)

        tk.Label(self.root, textvariable=self.edge_info_var, fg="gray").grid(
            row=row, column=2, columnspan=2, sticky="w", **padding
        )

        row += 1

        tk.Button(self.root, text="查询当前 TF", command=self.query_tf).grid(
            row=row, column=1, sticky="w", **padding
        )

        tk.Button(self.root, text="保存到 r2_nav_goals.yaml", command=self.save_goal).grid(
            row=row, column=2, sticky="w", **padding
        )

        row += 1

        tk.Label(self.root, text="x").grid(row=row, column=0, sticky="e", **padding)
        tk.Entry(self.root, textvariable=self.x_var, width=32, state="readonly").grid(
            row=row, column=1, sticky="w", **padding
        )

        tk.Label(self.root, text="y").grid(row=row, column=2, sticky="e", **padding)
        tk.Entry(self.root, textvariable=self.y_var, width=32, state="readonly").grid(
            row=row, column=3, sticky="w", **padding
        )

        row += 1

        tk.Label(self.root, text="z").grid(row=row, column=0, sticky="e", **padding)
        tk.Entry(self.root, textvariable=self.z_var, width=32, state="readonly").grid(
            row=row, column=1, sticky="w", **padding
        )

        tk.Label(self.root, text="yaw").grid(row=row, column=2, sticky="e", **padding)
        tk.Entry(self.root, textvariable=self.yaw_var, width=32, state="readonly").grid(
            row=row, column=3, sticky="w", **padding
        )

        row += 1

        tk.Label(self.root, text="状态").grid(row=row, column=0, sticky="e", **padding)
        tk.Label(self.root, textvariable=self.status_var, anchor="w", fg="blue").grid(
            row=row,
            column=1,
            columnspan=3,
            sticky="w",
            **padding,
        )

        row += 1

        help_text = (
            "使用方法：\n"
            "1. 启动真实定位/导航底座，让 TF 中存在 map -> chassis_base_link。\n"
            "2. 点击“加载 approach 点位列表”，下拉框会读取 meilin_map.yaml 里的所有 approach_goal_name。\n"
            "3. 选择一个点位名，例如 ENTRY_TO_B2_APPROACH。\n"
            "4. 把机器人移动到要标定的位置。\n"
            "5. 点击“查询当前 TF”。\n"
            "6. 点击“保存到 r2_nav_goals.yaml”。\n\n"
            "注意：child_frame 默认是 chassis_base_link，不要写成 chassis_base_kink。"
        )

        tk.Label(self.root, text=help_text, justify="left").grid(
            row=row,
            column=0,
            columnspan=4,
            sticky="w",
            padx=12,
            pady=16,
        )

    def select_map_yaml(self):
        path = filedialog.askopenfilename(
            title="选择 meilin_map.yaml",
            filetypes=[("YAML files", "*.yaml *.yml"), ("All files", "*.*")],
        )
        if path:
            self.map_yaml_path_var.set(path)
            self.load_goal_names_from_map_yaml()

    def select_nav_goals_yaml(self):
        path = filedialog.askopenfilename(
            title="选择 r2_nav_goals.yaml",
            filetypes=[("YAML files", "*.yaml *.yml"), ("All files", "*.*")],
        )
        if path:
            self.nav_goals_yaml_path_var.set(path)

    def load_yaml_file(self, path):
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)
        if data is None:
            data = {}
        return data

    def save_yaml_file(self, path, data):
        directory = os.path.dirname(path)
        if directory:
            os.makedirs(directory, exist_ok=True)

        with open(path, "w", encoding="utf-8") as f:
            yaml.safe_dump(
                data,
                f,
                allow_unicode=True,
                sort_keys=False,
                default_flow_style=False,
            )

    def load_goal_names_from_map_yaml(self):
        path = self.map_yaml_path_var.get().strip()

        if not path:
            messagebox.showerror("错误", "meilin_map.yaml 路径不能为空")
            return

        if not os.path.exists(path):
            messagebox.showerror("错误", f"找不到 meilin_map.yaml：\n{path}")
            return

        try:
            data = self.load_yaml_file(path)
            transitions = data.get("transitions", {})

            if not isinstance(transitions, dict):
                messagebox.showerror("错误", "meilin_map.yaml 中 transitions 格式不正确")
                return

            goal_names = []
            self.goal_name_to_edge = {}

            for edge_name, edge_info in transitions.items():
                if not isinstance(edge_info, dict):
                    continue

                approach_name = edge_info.get("approach_goal_name", "")

                if not approach_name:
                    continue

                goal_names.append(approach_name)
                self.goal_name_to_edge[approach_name] = edge_name

            goal_names = sorted(set(goal_names))

            if not goal_names:
                self.goal_name_combo["values"] = []
                self.goal_name_var.set("")
                self.edge_info_var.set("未找到 approach_goal_name")
                self.status_var.set("没有从 transitions 中找到 approach_goal_name")
                messagebox.showwarning(
                    "提示",
                    "没有从 meilin_map.yaml 的 transitions 中找到 approach_goal_name",
                )
                return

            self.goal_name_combo["values"] = goal_names

            current = self.goal_name_var.get().strip()
            if current in goal_names:
                self.goal_name_var.set(current)
            else:
                self.goal_name_var.set(goal_names[0])

            self.update_edge_info()
            self.status_var.set(f"已加载 {len(goal_names)} 个 approach 点位")

        except Exception as e:
            self.status_var.set(f"加载失败: {e}")
            messagebox.showerror("加载失败", str(e))

    def update_edge_info(self):
        goal_name = self.goal_name_var.get().strip()
        edge_name = self.goal_name_to_edge.get(goal_name, "")
        if edge_name:
            self.edge_info_var.set(f"来自 transition: {edge_name}")
        else:
            self.edge_info_var.set("")

    def on_goal_selected(self, _event=None):
        self.update_edge_info()

    def query_tf(self):
        parent = self.parent_frame_var.get().strip()
        child = self.child_frame_var.get().strip()

        if not parent:
            messagebox.showerror("错误", "父坐标系 parent_frame 不能为空")
            return

        if not child:
            messagebox.showerror("错误", "子坐标系 child_frame 不能为空")
            return

        try:
            pose = self.ros_node.query_pose(parent, child)
        except Exception as e:
            self.status_var.set(f"查询 TF 失败: {e}")
            messagebox.showerror("TF 查询失败", str(e))
            return

        self.current_pose = pose

        self.x_var.set(f"{pose['x']:.6f}")
        self.y_var.set(f"{pose['y']:.6f}")
        self.z_var.set(f"{pose['z']:.6f}")
        self.yaw_var.set(f"{pose['yaw']:.6f}")

        self.status_var.set(
            f"查询成功：{child} 在 {parent} 下的位置已读取"
        )

    def load_nav_goals_yaml(self, path):
        if not os.path.exists(path):
            return {"goals": {}}

        data = self.load_yaml_file(path)

        if "goals" not in data or data["goals"] is None:
            data["goals"] = {}

        if not isinstance(data["goals"], dict):
            raise RuntimeError("r2_nav_goals.yaml 中 goals 字段格式不正确")

        return data

    def save_goal(self):
        goal_name = self.goal_name_var.get().strip()
        yaml_path = self.nav_goals_yaml_path_var.get().strip()

        if not goal_name:
            messagebox.showerror(
                "错误",
                "请选择一个 approach 点位。若下拉框为空，请先点击“加载 approach 点位列表”。",
            )
            return

        if not yaml_path:
            messagebox.showerror("错误", "r2_nav_goals.yaml 路径不能为空")
            return

        if self.current_pose is None:
            answer = messagebox.askyesno(
                "还没有查询 TF",
                "当前还没有查询 TF，是否先查询一次再保存？",
            )
            if not answer:
                return

            self.query_tf()

            if self.current_pose is None:
                return

        try:
            data = self.load_nav_goals_yaml(yaml_path)

            if goal_name in data["goals"]:
                overwrite = messagebox.askyesno(
                    "覆盖确认",
                    f"点位 {goal_name} 已存在，是否覆盖？",
                )
                if not overwrite:
                    return

            pose = dict(self.current_pose)

            data["goals"][goal_name] = {
                "frame_id": pose["frame_id"],
                "x": round(pose["x"], 6),
                "y": round(pose["y"], 6),
                "z": round(pose["z"], 6),
                "yaw": round(pose["yaw"], 6),
            }

            self.save_yaml_file(yaml_path, data)

        except Exception as e:
            self.status_var.set(f"保存失败: {e}")
            messagebox.showerror("保存失败", str(e))
            return

        self.status_var.set(
            f"保存成功：{goal_name} -> {yaml_path}"
        )
        messagebox.showinfo(
            "保存成功",
            f"已保存点位：{goal_name}",
        )

    def on_close(self):
        self.root.destroy()

    def run(self):
        self.root.mainloop()


def main():
    rclpy.init()

    node = GoalRecorderNode()

    executor = SingleThreadedExecutor()
    executor.add_node(node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        gui = GoalRecorderGui(node)
        gui.run()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()