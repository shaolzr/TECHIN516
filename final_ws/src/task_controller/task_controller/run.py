import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion
from .gen3lite_pymoveit2 import Gen3LiteArm, Gen3LiteGripper
import time
import csv
import sys

# Load pose data from CSV file
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

# Execute arm and gripper movements
def execute_movements(arm, gripper, movements):
    for action in movements:
        if isinstance(action, Pose):
            success = arm.inverse_kinematic_movement(action)
            if success:
                print(f"Successfully moved to position: {action.position}")
            else:
                print(f"Failed to move to position: {action.position}")
        elif isinstance(action, float):
            gripper.move_to_position(action)
            print(f"Gripper moved to position: {action}")
        time.sleep(1)

# Main function
def main():
    rclpy.init()
    arm = Gen3LiteArm()
    gripper = Gen3LiteGripper()

    print("Process started...")

    # Load pose data from CSV
    poses = load_poses_from_csv("/home/shaolzr@netid.washington.edu/TECHIN516/final_ws/src/task_controller/task_controller/put.csv")

    # Define movement sequence
    movements = [
        poses['grasp_up'],
        0.1,
        poses['grasp_up1'],
        poses['grasp'],
        0.8,  # Grip the gripper at the 'grasp' position
        poses['grasp_mid1'],
        poses['grasp_mid2_down'],
        poses['grasp_put'],
        0.1  # Release the gripper at the 'grasp_put' position
    ]

    # Execute movements
    execute_movements(arm, gripper, movements)

    # Shutdown devices
    #gripper.shutdown()
    #arm.shutdown()
    #rclpy.shutdown()

    print("Process completed. Exiting now.")
    #sys.exit(0)  # Immediately exit the program with a success status code

if __name__ == '__main__':
    main()

