
import time
import numpy as np
from scipy.spatial.transform import Rotation as R

import rospy
from std_msgs.msg import Int32, Float32, Float32MultiArray
from sensor_msgs.msg import Imu, JointState
from geometry_msgs.msg import Twist, WrenchStamped
from nav_msgs.msg import Odometry

class imuSensor2Eular:
    def __init__(self):
        rospy.init_node('imuSensor2Eular')
        self.loop_rate = 500
        self.eular_angle_msg = Odometry()
        rospy.Subscriber("/imu/data", Imu, self.subPhidgetImu)

    def subPhidgetImu(self, msg: Imu):
        eular_angle = R.from_quat([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]).as_euler('xyz').copy()

        self.eular_angle_msg.pose.pose.orientation.x = eular_angle[0] * 180.0/3.1415
        self.eular_angle_msg.pose.pose.orientation.y = eular_angle[1] * 180.0/3.1415
        self.eular_angle_msg.pose.pose.orientation.z = eular_angle[2] * 180.0/3.1415

        self.joint_state_pub = rospy.Publisher('/imu_phidget_Spacial/eular_angle', Odometry, queue_size=1)
        self.joint_state_pub.publish(self.eular_angle_msg)
        # print("sub PhidgetImu ", eular_angle)

    def run(self):
        while True:
            try:
                time.sleep(1.0/self.loop_rate)
            except KeyboardInterrupt:
                break

if __name__ == "__main__":
    lce = imuSensor2Eular()
    lce.run()
