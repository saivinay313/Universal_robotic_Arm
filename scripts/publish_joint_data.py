#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Thu Jun 17 19:18:27 2021

@author: vinay
"""

import rospy
from math import pi 
from std_msgs.msg import Float64
import numpy as np
from UR5_inverse_kinematics import invKine

desired_pose = np.matrix([[1,0,0,2],
                          [0,1,0,3],
                          [0,0,1,0],
                          [0,0,0,1],
                          ])
theta = invKine(desired_pose)
theta_list = theta[:,0]

joint_position1  =theta_list[0]*(180/pi)
joint_position2  =theta_list[1]*(180/pi)
joint_position3  =theta_list[2]*(180/pi)
joint_position4  =theta_list[3]*(180/pi)
joint_position5  =theta_list[4]*(180/pi)
joint_position6  =theta_list[5]*(180/pi)

def joint_publisher():
    pub1 = rospy.Publisher("/simple_robot/joint1_position_controller/command",Float64,queue_size=10)
    pub2 = rospy.Publisher("/simple_robot/joint2_position_controller/command",Float64,queue_size=10)
    pub3 = rospy.Publisher("/simple_robot/joint3_position_controller/command",Float64,queue_size=10)
    pub4 = rospy.Publisher("/simple_robot/joint4_position_controller/command",Float64,queue_size=10)
    pub5 = rospy.Publisher("/simple_robot/joint5_position_controller/command",Float64,queue_size=10)
    pub6 = rospy.Publisher("/simple_robot/joint6_position_controller/command",Float64,queue_size=10)
    rospy.init_node('joint_publisher',anonymous=True)
    
    while not rospy.is_shutdown():
        pub1.publish(joint_position1*(pi/180))
        pub2.publish(joint_position2*(pi/180))
        pub3.publish(joint_position3*(pi/180))
        pub4.publish(joint_position4*(pi/180))
        pub5.publish(joint_position5*(pi/180))
        pub6.publish(joint_position6*(pi/180))
        rospy.loginfo(joint_position1)
        rospy.loginfo(joint_position3)
        rospy.loginfo(joint_position4)
        rospy.loginfo(joint_position5)
        rospy.loginfo(joint_position6)
        
    
if __name__ == "__main__":
    try:
        joint_publisher()
    except rospy.ROSInterruptException:
        pass