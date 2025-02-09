#!/usr/bin/env python3

import rospy
from kinematics.Mecanum_kinematics import KinematicsMecanum
from odom.odomClass import Odom
from imu_message.SensorIMU import SensorIMU

from sensor_msgs.msg import Imu
from imu_message.imu_bno055_api import BoschIMU


yaw_degree=0
#torta
wheel_radius = 0.04  
lx = 0.175  
ly = 0.07 
#shato
# wheel_radius = 0.04  
# lx = 0.15  
# ly = 0.15
kinematics = KinematicsMecanum(lx, ly)
odometry = Odom()
imu=BoschIMU(port="/dev/ttyUSB1")
imu_msg=Imu()
imu_sensor = SensorIMU(imu,imu_msg)

def main():
    rospy.init_node("mainOdom")
    


    rate = rospy.Rate(50)
    while not rospy.is_shutdown():

        wheels_velocity= odometry.rpm_to_angular(odometry.wheels_rpm)

        vx, vy, omega= kinematics.mecanum_forward(*wheels_velocity)

        yaw_rad,yaw_degree = imu_sensor.get_yaw()
        dt = odometry.calculate_dt()
        odometry.update_world_frame(vx,vy,omega,yaw_rad,dt)
        odometry.publish_odometry()
        rospy.loginfo(f"yaw:{yaw_degree}")
        rate.sleep()

if __name__ == "__main__":
    main()