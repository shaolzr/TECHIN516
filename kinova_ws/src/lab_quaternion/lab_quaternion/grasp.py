import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion
from .gen3lite_pymoveit2 import Gen3LiteArm, Gen3LiteGripper
import time
import csv


def load_poses_from_csv(file_path):
    with open(file_path, "r") as csvfile:
        reader = csv.DictReader(csvfile)
        poses = {
            row['name']: Pose(
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
            for row in reader
        }
    return poses


def execute_movements(arm, gripper, movements):
    for action in movements:
        if isinstance(action, Pose):
            arm.inverse_kinematic_movement(action)
        elif isinstance(action, float):
            gripper.move_to_position(action)
        time.sleep(1)


def main():
    rclpy.init()
    arm = Gen3LiteArm()
    gripper = Gen3LiteGripper()
    arm.go_vertical()
    gripper.move_to_position(0.1)

    poses = load_poses_from_csv("/home/shaolzr@netid.washington.edu/TECHIN516/kinova_ws/src/lab_quaternion/lab_quaternion/grasp.csv")

    movements = [
        poses['grasp_up'],
        poses['grasp'],
        0.8,  # Close gripper
        poses['grasp_mid'],
        poses['grasp_mid1'],
        poses['grasp_put'],
        0.1  # Open gripper
    ]

    execute_movements(arm, gripper, movements)

    gripper.shutdown()
    arm.shutdown()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
