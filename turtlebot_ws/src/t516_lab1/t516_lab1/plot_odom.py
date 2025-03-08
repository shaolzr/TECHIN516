#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt

class OdomSubscriber(Node):

    def __init__(self):
        super().__init__('odom_subscriber')
        self.subscription = self.create_subscription(
            Odometry,            # 消息类型
            '/odom',             # 订阅的主题
            self.odom_callback,  # 回调函数
            10                   # 队列大小
        )
        self.subscription  # prevent unused variable warning

        # Lists to store the x and y data
        self.x_data = []
        self.y_data = []

    def odom_callback(self, msg):
        # Extract the x and y positions
        position = msg.pose.pose.position  # 从消息中提取位置
        x = position.x                    # 提取 x 坐标
        y = position.y                    # 提取 y 坐标

        # Log the position
        self.get_logger().info(f"Position -> x: {x}, y: {y}")

        # Append the x and y positions to the lists
        self.x_data.append(x)
        self.y_data.append(y)

    def plot_data(self):
        # Plot x and y data
        plt.figure()
        plt.plot(self.x_data, self.y_data, label='Odometry Path')
        plt.title('Odometry X-Y Path')
        plt.xlabel('X Position')
        plt.ylabel('Y Position')
        plt.legend()
        plt.grid(True)
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    odom_subscriber = OdomSubscriber()

    try:
        rclpy.spin(odom_subscriber)
    except KeyboardInterrupt:
        # On shutdown, plot the data
        odom_subscriber.plot_data()

    odom_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

