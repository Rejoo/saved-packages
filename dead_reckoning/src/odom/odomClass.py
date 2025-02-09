#!/usr/bin/env python3

import math
import rospy
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf.transformations import quaternion_from_euler 
import tf2_ros


class Odom:
    def __init__(self):
        self.odom_seq = 0
        self.theta = 0
        self.x = 0
        self.y = 0
        self.vx = 0
        self.vy = 0
        self.omega = 0
        self.yaw_rad = 0
        self.wheels_rpm =[0,0,0,0]
        self.x, self.y, self.theta = 0.0, 0.0, 0.0
        self.vx, self.vy, self.omega = 0.0, 0.0, 0.0
        self.last_time = None
        
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.odom_broadcaster = tf2_ros.TransformBroadcaster()
        rospy.Subscriber('/Espeeds', Float32MultiArray, self.encoder_callback)

    def calculate_dt(self):
        current_time = rospy.Time.now()
        if self.last_time is None:
            self.last_time = current_time
        dt = (current_time - self.last_time).to_sec()
        self.last_time = current_time
        return dt

    def encoder_callback(self, msg:Float32MultiArray):
        self.wheels_rpm = msg.data

        return self.wheels_rpm

    def rpm_to_angular(self,wheels_rpm):
        wheels_velocity = [(rpm * 2 * math.pi ) / 60 for rpm in wheels_rpm]
        return wheels_velocity

    def update_world_frame(self,vx,vy,omega,yaw_rad,dt):
        self.vx = vx
        self.vy = vy
        self.omega = omega
    
        vx_world = (vx * math.cos(yaw_rad)) - (vy * math.sin(yaw_rad))
        vy_world = (vx * math.sin(yaw_rad)) + (vy * math.cos(yaw_rad))

        dx = vx_world * dt
        dy = vy_world * dt
        dtheta = omega * dt

        self.x += dx
        self.y += dy
        self.theta += dtheta

    def publish_odometry(self):    
        odom_msg = Odometry()
        odom_msg.header.seq = self.odom_seq
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"

        odom_msg.twist.twist.linear.x = self.vx
        odom_msg.twist.twist.linear.y = self.vy
        odom_msg.twist.twist.angular.z = self.omega

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.orientation.z = math.sin(self.yaw_rad  / 2.0)
        odom_msg.pose.pose.orientation.w = math.cos(self.yaw_rad  / 2.0)


        odom_trans = TransformStamped()

        odom_trans.header.stamp = rospy.Time.now()
        odom_trans.header.frame_id = "odom" 
        odom_trans.child_frame_id = "base_link"  

        odom_trans.transform.translation.x = self.x
        odom_trans.transform.translation.y = self.y
        odom_trans.transform.translation.z = 0.0

        odom_quat = quaternion_from_euler(0, 0, self.yaw_rad )

        odom_trans.transform.rotation.x = odom_quat[0]
        odom_trans.transform.rotation.y = odom_quat[1]
        odom_trans.transform.rotation.z = odom_quat[2]
        odom_trans.transform.rotation.w = odom_quat[3]

        self.odom_broadcaster.sendTransform(odom_trans)
        
        self.odom_pub.publish(odom_msg)
        self.odom_seq += 1


