#!/usr/bin/env python3
import time
import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import csv
from geometry_msgs.msg import Pose, Point, Quaternion

# ✅ 修正 Import 错误
try:
    from gen3lite_pymoveit2 import Gen3LiteArm, Gen3LiteGripper
except ModuleNotFoundError:
    try:
        from task_controller.gen3lite_pymoveit2 import Gen3LiteArm, Gen3LiteGripper
    except ModuleNotFoundError:
        print("❌ Error: Cannot import gen3lite_pymoveit2. Check your PYTHONPATH or package installation.")
        sys.exit(1)

###############################################################################
# FORWARD MOTION
###############################################################################
class TurtleBotForwardController(Node):
    def __init__(self, speed=0.1, distance=3.3):
        super().__init__('turtlebot_forward_controller')
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.speed = speed
        self.target_distance = distance
        self.movement_complete = False

    def move_forward(self):
        print("🚀 Moving forward...")
        twist = Twist()
        twist.linear.x = self.speed
        duration = self.target_distance / abs(self.speed)
        start_time = time.time()

        while not self.movement_complete and (time.time() - start_time) < duration:
            self.cmd_vel_publisher.publish(twist)
            traveled_distance = abs(self.speed) * (time.time() - start_time)
            print(f"Moved: {traveled_distance:.2f} meters", end='\r')
            time.sleep(0.1)

        twist.linear.x = 0.0
        self.cmd_vel_publisher.publish(twist)
        self.movement_complete = True
        print("\n✅ Finished moving forward.")

###############################################################################
# BACKWARD MOTION
###############################################################################
class TurtleBotBackwardController(Node):
    def __init__(self, speed=-0.1, distance=3.3):
        super().__init__('turtlebot_backward_controller')
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.speed = speed
        self.target_distance = distance
        self.movement_complete = False

    def move_backward(self):
        print("🔙 Moving backward...")
        twist = Twist()
        twist.linear.x = self.speed
        duration = self.target_distance / abs(self.speed)
        start_time = time.time()

        while not self.movement_complete and (time.time() - start_time) < duration:
            self.cmd_vel_publisher.publish(twist)
            traveled_distance = abs(self.speed) * (time.time() - start_time)
            print(f"Moved: {traveled_distance:.2f} meters", end='\r')
            time.sleep(0.1)

        twist.linear.x = 0.0
        self.cmd_vel_publisher.publish(twist)
        self.movement_complete = True
        print("\n✅ Finished moving backward.")

###############################################################################
# LOAD POSES FROM CSV
###############################################################################
def load_poses_from_csv(file_path):
    with open(file_path, "r") as csvfile:
        reader = csv.DictReader(csvfile)
        poses = {}
        for row in reader:
            name = row['name']
            poses[name] = Pose(
                position=Point(
                    x=float(row['pos_x']),
                    y=float(row['pos_y']),
                    z=float(row['pos_z'])
                ),
                orientation=Quaternion(
                    x=float(row['x']),
                    y=float(row['y']),
                    z=float(row['z']),
                    w=float(row['w'])
                )
            )
        return poses

###############################################################################
# EXECUTE ARM AND GRIPPER
###############################################################################
def execute_arm_and_gripper(arm, gripper, movements):
    """
    逐步执行机械臂和夹爪动作：
      - 如果是 Pose，调用 inverse_kinematic_movement()
      - 如果是 float，调用 move_to_position()
      - 每步等待 1 秒
    """
    for action in movements:
        if isinstance(action, Pose):
            print(f"\n[ARM] Moving to {action.position}")
            success = arm.inverse_kinematic_movement(action)
            print(f"[ARM] Result: {success}")
            if success is None or not success:
                print("[ARM] ❌ Failed to move. Skipping to next action.")
        elif isinstance(action, float):
            print(f"\n[GRIPPER] Moving to position {action}")
            result = gripper.move_to_position(action)
            print(f"[GRIPPER] Result: {result}")
        time.sleep(1)  # 保持 1 秒等待

    print("\n✅ Arm and Gripper execution finished.")

###############################################################################
# RUN ROBOT ARM SEQUENCE
###############################################################################
def run_robot_arm_sequence():
    print("🤖 Running robot arm sequence...")

    # 创建机械臂和夹爪实例
    arm = Gen3LiteArm()
    gripper = Gen3LiteGripper()

    # 加载 CSV
    csv_path = "/home/shaolzr@netid.washington.edu/TECHIN516/final_ws/src/task_controller/task_controller/put.csv"
    poses = load_poses_from_csv(csv_path)

    # 机械臂执行序列
    movements = [
        poses['grasp_up'],
        0.1,  # 部分关闭夹爪
        poses['grasp_up1'],
        poses['grasp'],
        0.8,  # 完全夹住物体
        poses['grasp_mid1'],
        poses['grasp_mid2_down'],
        poses['grasp_put'],
        0.1   # 释放物体
    ]

    # 执行动作
    execute_arm_and_gripper(arm, gripper, movements)

    # 关闭机械臂
    #gripper.shutdown()
    #arm.shutdown()

    print("✅ Finished robot arm sequence.")

###############################################################################
# MAIN FUNCTION
###############################################################################
def main():
    rclpy.init()

    # 1️⃣ 移动前进
    forward_node = TurtleBotForwardController(speed=0.1, distance=3.3)
    forward_node.move_forward()
    forward_node.destroy_node()

    # 2️⃣ 运行机械臂
    run_robot_arm_sequence()

    # 3️⃣ 立刻执行后退
    print("\n🔙 Now moving backward immediately!")
    backward_node = TurtleBotBackwardController(speed=-0.1, distance=3.3)
    backward_node.move_backward()
    backward_node.destroy_node()

    # 清理
    rclpy.shutdown()
    sys.exit(0)

if __name__ == '__main__':
    main()

