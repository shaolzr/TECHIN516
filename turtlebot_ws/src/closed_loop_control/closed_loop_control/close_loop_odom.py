#!/usr/bin/env python3

import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node

class OdomCloseLoop(Node):
    def __init__(self):
        super().__init__('close_loop_odom')
        # debug mode
        self.debug = True
        
        # TODO: 订阅 /odom 话题
        self.sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        # TODO: 添加发布器，发送速度控制指令
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.position_x_list = []
        self.motion_move = Twist()
        self.motion_stop = Twist()

        # 设定前进速度
        self.motion_move.linear.x = 0.15
        # 停止速度
        self.motion_stop.linear.x = 0.0

    def odom_callback(self, msg):
        position_x = msg.pose.pose.position.x
        self.position_x_list.append(position_x)

        # 确保列表至少有 2 个数据点
        if len(self.position_x_list) > 2:
            # TODO: 检查最后一个记录的位置是否大于起始位置+1.5m
            if self.position_x_list[-1] >= self.position_x_list[0] + 1.5:
                self.pub.publish(self.motion_stop)  # 停止机器人
                self.get_logger().info("✅ Arrived, Stop")
                rclpy.shutdown()
            else:
                self.pub.publish(self.motion_move)  # 继续移动
                if self.debug:
                    self.get_logger().info(f"📏 Initial postion: {msg.pose.pose}")

def main(args=None):
    rclpy.init(args=args)
    odom_cl = OdomCloseLoop()
    rclpy.spin(odom_cl)
    odom_cl.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

