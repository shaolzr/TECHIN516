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
        Rotate TurtleBot by a specified angle.
        :param angle: Rotation angle (degrees, positive = left, negative = right)
        :param angular_speed: Angular speed (rad/s)
        """
        print(f"🔄 Rotating {'left' if angle > 0 else 'right'} {abs(angle)} degrees...")
        
        twist = Twist()
        twist.angular.z = angular_speed if angle > 0 else -abs(angular_speed)  

        # Calculate rotation duration
        radians = math.radians(abs(angle))
        duration = radians / abs(angular_speed)

        start_time = time.time()
        while time.time() - start_time < duration:
            self.cmd_vel_publisher.publish(twist)
            time.sleep(0.1)

        # Stop rotation
        twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)
        print("✅ Finished rotation.")

    def move_forward(self, distance, speed=0.1):
        """
        Move TurtleBot forward or backward.
        :param distance: Distance to move (meters, negative = backward)
        :param speed: Linear speed (m/s, must be positive)
        """
        print(f"🚀 Moving {'forward' if distance > 0 else 'backward'} {abs(distance)} meters...")

        twist = Twist()
        twist.linear.x = speed if distance > 0 else -speed  # Support backward movement

        # Calculate move duration
        duration = abs(distance) / speed

        start_time = time.time()
        while time.time() - start_time < duration:
            self.cmd_vel_publisher.publish(twist)
            time.sleep(0.1)

        # Stop movement
        twist.linear.x = 0.0
        self.cmd_vel_publisher.publish(twist)
        print("✅ Finished moving.")

def main():
    rclpy.init()
    node = TurtleBotTurnAndMove()

    # 🛠️ Define movement sequence (Negative distance means moving backward)
    movements = [
        {"angle": 0, "distance": -0.9},  # Move backward 0.8m
        {"angle": 42, "distance": -1.2}, # Right turn 90° → Move forward 0.8m
        {"angle": -90, "distance": 1.2}, # Left turn 45° → Move backward 1.2m
        {"angle": -135, "distance": 1.87},  # Right turn 90° → Move forward 1.2m
    ]

    # ✅ Execute movements
    for i, move in enumerate(movements):
        print(f"\n🟢 Step {i+1}: Turn {move['angle']}° → Move {move['distance']}m")
        node.turn(angle=move["angle"])
        node.move_forward(distance=move["distance"])

    # Cleanup and exit
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

