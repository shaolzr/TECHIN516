import numpy as np
from pyquaternion import Quaternion
from geometry_msgs.msg import Pose
from .gen3lite_pymoveit2 import Gen3LiteArm
import rclpy

def main():
    # Load position data from CSV file
    data_path = '/home/shaolzr@netid.washington.edu/TECHIN516/kinova_ws/src/lab_quaternion/lab_quaternion/data.csv'
    data = np.loadtxt(data_path, delimiter=',')

    # Interpolate quaternion orientations
    qStart = Quaternion(array=np.array([0, 0.576, 0.002, 0.434]))
    qEnd = Quaternion(array=np.array([0, 0.388, 0.598, 0.535]))
    qList = [q.elements for q in Quaternion.intermediates(qStart, qEnd, len(data)-2, include_endpoints=True)]

    # Initialize ROS2 and Gen3LiteArm
    rclpy.init()
    arm = Gen3LiteArm()

    # Move arm to interpolated positions
    for i in range(len(data)):
        pose = Pose()
        pose.position.x = data[i][0]
        pose.position.y = data[i][1]
        pose.position.z = data[i][2]
        pose.orientation.x = qList[i][0]
        pose.orientation.y = qList[i][1]
        pose.orientation.z = qList[i][2]
        pose.orientation.w = qList[i][3]
        arm.inverse_kinematic_movement(pose)

    # Shutdown ROS2 and arm
    rclpy.shutdown()
    arm.shutdown()

if __name__ == '__main__':
    main()

