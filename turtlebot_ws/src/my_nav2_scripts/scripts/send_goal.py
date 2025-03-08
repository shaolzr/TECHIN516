import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import time

class GoalPublisher(Node):
    def __init__(self):
        super().__init__('goal_publisher')
        self.publisher = self.create_publisher(PoseStamped, '/navigate_to_pose', 10)
        time.sleep(2)  # 等待ROS2系统稳定

    def send_goal(self, x, y, yaw=0.0):
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = "map"
        goal_msg.pose.position.x = x
        goal_msg.pose.position.y = y
        goal_msg.pose.orientation.w = 1.0  # 机器人面向默认方向

        self.publisher.publish(goal_msg)
        self.get_logger().info(f"Sent goal: x={x}, y={y}")

def main(args=None):
    rclpy.init(args=args)
    node = GoalPublisher()
    node.send_goal(-2.0, -0.5)  # 发送目标点
    rclpy.shutdown()

if __name__ == '__main__':
    main()

