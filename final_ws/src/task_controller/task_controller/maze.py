#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import math

class TurtleBotTurnAndMove(Node):
    def __init__(self):
        super().__init__('turtlebot_turn_and_move')
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

    def turn(self, angle, angular_speed=0.2):
        """
        让 TurtleBot 旋转指定角度（正数左转，负数右转）
        :param angle: 旋转角度（度）
        :param angular_speed: 角速度（rad/s），正数左转，负数右转
        """
        print(f"🔄 Rotating {'left' if angle > 0 else 'right'} {abs(angle)} degrees...")
        
        twist = Twist()
        twist.angular.z = angular_speed if angle > 0 else -abs(angular_speed)  # 自动匹配方向

        # 计算旋转时间
        radians = math.radians(abs(angle))
        duration = radians / abs(angular_speed)

        start_time = time.time()
        while time.time() - start_time < duration:
            self.cmd_vel_publisher.publish(twist)
            time.sleep(0.1)

        # 停止旋转
        twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)
        print("✅ Finished rotation.")

    def move_forward(self, distance, speed=0.1):
        """
        让 TurtleBot 向前移动指定距离
        :param distance: 移动的距离（米）
        :param speed: 线速度（米/秒）
        """
        print(f"🚀 Moving forward {distance} meters...")
        
        twist = Twist()
        twist.linear.x = abs(speed)

        # 计算移动时间
        duration = distance / speed

        start_time = time.time()
        while time.time() - start_time < duration:
            self.cmd_vel_publisher.publish(twist)
            time.sleep(0.1)

        # 停止移动
        twist.linear.x = 0.0
        self.cmd_vel_publisher.publish(twist)
        print("✅ Finished moving forward.")

def main():
    rclpy.init()
    node = TurtleBotTurnAndMove()

    # 🛠️ 这里可以修改 **不同的旋转角度和前进距离**
    movements = [
        {"angle": -45, "distance": 1.65},  # 旋转 45°，前进 1m
        {"angle": 135, "distance": 1.2}, # 右转 30°，前进 2m
        {"angle": -87, "distance": 1.2},  # 左转 90°，前进 0.5m
        {"angle": -45, "distance": 0.9}, # 右转 60°，前进 1.5m
    ]

    # ✅ 逐步执行所有旋转 & 移动
    for i, move in enumerate(movements):
        print(f"\n🟢 Step {i+1}: Turn {move['angle']}° → Move {move['distance']}m")
        node.turn(angle=move["angle"])
        node.move_forward(distance=move["distance"])

    # 清理并退出
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

