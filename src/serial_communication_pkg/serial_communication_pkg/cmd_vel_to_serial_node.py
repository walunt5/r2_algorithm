#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time
import struct
from threading import Lock

class CmdVelToSerial(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_serial_node')
        
        # 参数配置
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('linear_scale', 1000.0)
        self.declare_parameter('angular_scale', 1000.0)
        self.declare_parameter('publish_rate', 100.0)  # 100Hz = 10ms

        # 速度限幅参数
        self.declare_parameter('max_vx', 0.08)
        self.declare_parameter('max_vy', 0.08)
        self.declare_parameter('max_wz', 0.20)

        # 指令超时时间
        self.declare_parameter('cmd_timeout', 0.3)
        
        # 获取参数
        serial_port = self.get_parameter('serial_port').value
        baud_rate = self.get_parameter('baud_rate').value
        self.linear_scale = self.get_parameter('linear_scale').value
        self.angular_scale = self.get_parameter('angular_scale').value
        publish_rate = self.get_parameter('publish_rate').value
        
        self.max_vx = self.get_parameter('max_vx').value
        self.max_vy = self.get_parameter('max_vy').value
        self.max_wz = self.get_parameter('max_wz').value
        self.cmd_timeout = self.get_parameter('cmd_timeout').value
        # 线程锁，保护共享数据
        self.data_lock = Lock()
        
        # 当前速度指令
        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_wz = 0.0
        self.last_cmd_time = self.get_clock().now()

        # 统计信息
        self.sent_count = 0
        self.error_count = 0
        self.last_stat_time = self.get_clock().now()
        
        # 初始化串口
        self.serial = None
        try:
            self.serial = serial.Serial(
                port=serial_port,
                baudrate=baud_rate,
                timeout=0.01  # 短的超时时间
            )
            time.sleep(2)  # 等待串口稳定
            self.get_logger().info(f'成功打开串口: {serial_port}, 波特率: {baud_rate}')
        except Exception as e:
            self.get_logger().error(f'打开串口失败: {e}')
            raise RuntimeError(f'打开串口失败: {e}')
        
        # 创建订阅者
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # 创建定时器，10ms周期
        self.timer = self.create_timer(1.0/publish_rate, self.timer_callback)
        
        
        
        self.get_logger().info(f'cmd_vel转串口节点已启动，发送频率: {publish_rate}Hz')
        
    def cmd_vel_callback(self, msg):
        """接收速度指令并更新当前速度"""
        with self.data_lock:
            self.current_vx = msg.linear.x
            self.current_vy = msg.linear.y
            self.current_wz = msg.angular.z
            self.last_cmd_time = self.get_clock().now()
        
        self.get_logger().debug(f'收到速度指令: Vx={self.current_vx:.3f}, Vy={self.current_vy:.3f}, Wz={self.current_wz:.3f}')
    
    def clamp(self, value, min_value, max_value):
        """限制数值在指定范围内"""
        return max(min(value, max_value), min_value)

    def timer_callback(self):
        """定时器回调，每10ms执行一次"""
        if self.serial is None or not self.serial.is_open:
            return
        
        # 检查指令是否超时
        current_time = self.get_clock().now()
        time_since_last_cmd = (current_time - self.last_cmd_time).nanoseconds / 1e9
        
        if time_since_last_cmd > self.cmd_timeout:
            # 指令超时，停止运动
            vx_int, vy_int, wz_int = 0, 0, 0
            timeout_flag = True
        else:
            # 使用当前速度指令
            with self.data_lock:
                vx = self.clamp(self.current_vx, -self.max_vx, self.max_vx)
                vy = self.clamp(self.current_vy, -self.max_vy, self.max_vy)
                wz = self.clamp(self.current_wz, -self.max_wz, self.max_wz)

                vx_int = int(vx * self.linear_scale)
                vy_int = int(vy * self.linear_scale)
                wz_int = int(wz * self.angular_scale)

            timeout_flag = False
        
        # 限制数值范围
        vx_int = max(min(vx_int, 32767), -32768)
        vy_int = max(min(vy_int, 32767), -32768)
        wz_int = max(min(wz_int, 32767), -32768)
        
        # 构建并发送数据帧
        try:
            frame = self.build_frame(vx_int, vy_int, wz_int)
            bytes_written = self.serial.write(frame)
            
            if bytes_written == len(frame):
                self.sent_count += 1
                if timeout_flag and self.sent_count % 10 == 0:  # 每100ms打印一次超时警告
                    self.get_logger().warn('速度指令超时，发送停止指令')
            else:
                self.error_count += 1
                self.get_logger().error(f'串口发送不完整: {bytes_written}/{len(frame)} 字节')
                
        except Exception as e:
            self.error_count += 1
            self.get_logger().error(f'串口发送失败: {e}')
        
        # 每5秒打印一次统计信息
        if (current_time - self.last_stat_time).nanoseconds / 1e9 >= 5.0:
            self.print_statistics()
            self.last_stat_time = current_time
    
    def print_statistics(self):
        """打印通信统计信息"""
        total_frames = self.sent_count + self.error_count
        success_rate = (self.sent_count / total_frames * 100) if total_frames > 0 else 0
        self.get_logger().info(
            f'通信统计 - 成功: {self.sent_count}, 错误: {self.error_count}, '
            f'成功率: {success_rate:.1f}%'
        )
    
    def int16_to_bytes(self, value):
        """将16位有符号整数转换为2字节（小端序）"""
        return list(struct.pack('<h', value))  # '<h' 表示小端16位有符号整数
    
    def build_frame(self, vx_int, vy_int, wz_int):
        """构建数据帧"""
        frame = bytearray()
        
        # 帧头
        frame.append(0x5A)  # 帧头
        
        # 数据部分 - 小端序
        vx_bytes = self.int16_to_bytes(vx_int)
        vy_bytes = self.int16_to_bytes(vy_int)
        wz_bytes = self.int16_to_bytes(wz_int)
        
        # 组合所有字节
        data_bytes = vx_bytes + vy_bytes + wz_bytes
        frame.extend(data_bytes)
        
        # 计算校验和（数据部分的6个字节）
        checksum = 0
        for byte in data_bytes:
            checksum ^= byte
        frame.append(checksum)
        
        # 帧尾
        frame.append(0xA5)  # 帧尾
        
        return frame
    
    def destroy_node(self):
        """节点销毁时的清理工作"""
        # 发送停止指令
        if self.serial and self.serial.is_open:
            try:
                stop_frame = self.build_frame(0, 0, 0)
                self.serial.write(stop_frame)
                self.get_logger().info('发送停止指令')
            except:
                pass
            
            self.serial.close()
            self.get_logger().info('串口已关闭')
        
        # 打印最终统计
        self.print_statistics()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToSerial()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('收到键盘中断信号')
    except Exception as e:
        node.get_logger().error(f'节点运行异常: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

    