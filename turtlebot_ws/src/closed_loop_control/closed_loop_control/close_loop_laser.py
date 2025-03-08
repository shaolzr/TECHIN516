#!/usr/bin/env python3

import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node

class LaserCloseLoop(Node):
    def __init__(self):
        super().__init__('laser_closeloop')
        self.debug = True

        # 订阅 /scan 话题，获取激光雷达数据
        self.sub = self.create_subscription(
            LaserScan, 
            '/scan', 
            self.scan_callback, 
            10
        )
        
        # 发布到 /cmd_vel 话题，控制机器人移动
        self.pub = self.create_publisher(
            Twist, 
            '/cmd_vel', 
            10
        )

        self.motion_move = Twist()
        self.motion_stop = Twist()
        
        # 设置机器人前进的速度
        self.motion_move.linear.x = 0.15  # 确保机器人是向前的速度
        
        # 设置停止的速度
        self.motion_stop.linear.x = 0.0

        self.initial_distance = None  # 记录起始前方障碍物距离

    def scan_callback(self, msg):
        """
        订阅 /scan 话题的数据并控制机器人前进或停止
        """
        # 选择前方 10° 范围内的扫描数据
        front_index = len(msg.ranges) // 2
        front_angles = msg.ranges[front_index - 5 : front_index + 5]
        
        # 获取前方的最小值，避免单点噪声
        current_front_value = min(front_angles)

        # 过滤掉无效数据（NaN 或 inf）
        if np.isinf(current_front_value) or np.isnan(current_front_value):
            return  # 忽略无效数据

        # 记录初始前方距离（仅设置一次）
        if self.initial_distance is None:
            self.initial_distance = current_front_value
            self.get_logger().info(f"Initial front distance recorded: {self.initial_distance:.2f} meters")
            return  # 退出回调，等待下次数据

        # 计算机器人前进的距离（确保为正值）
        distance_traveled = self.initial_distance - current_front_value

        # 机器人前进 1.5m 后停止（修正：无论正负都停止）
        if abs(distance_traveled) >= 1.5:
            self.pub.publish(self.motion_stop)
            self.get_logger().info(f"Reached goal! Distance traveled: {distance_traveled:.2f} meters. Stopping...")
            rclpy.shutdown()
        else:
            self.pub.publish(self.motion_move)
            if self.debug:
                self.get_logger().info(f"Value ahead: {current_front_value:.2f} meters")
                self.get_logger().info(f"Distance traveled: {distance_traveled:.2f} meters")

def main(args=None):
    """
    ROS 2 节点的主函数
    """
    rclpy.init(args=args)
    scan_cl = LaserCloseLoop()
    rclpy.spin(scan_cl)
    scan_cl.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

