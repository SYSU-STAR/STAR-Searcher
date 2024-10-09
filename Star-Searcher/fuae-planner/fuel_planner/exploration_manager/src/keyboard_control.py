#! /usr/bin/env python
import pygame
from pygame.locals import *
import time
import sys
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState
import numpy as np
import math
import tf.transformations as tft
def main():
    rospy.init_node('keyboard_control')
    uav_state = -1 #key value
    key_axes = [0, 0, 0, 0, 0, 0, 0, 0] #key value
    # initialize pygame to get keyboard event
    pygame.init()
    window_size = Rect(0, 0, 691, 272)
    screen = pygame.display.set_mode(window_size.size)

    img = pygame.image.load("./files/keyboard3.jpg")
    key_pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)
    get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

    rate = rospy.Rate(50)

    body_pose_msg = ModelState() #uav body pose
    last_body_pose_msg = ModelState() #
    pub_pose_msg = ModelState()
    pub_pose_msg.model_name = 'ardrone'
    pub_pose_msg.reference_frame = 'world'
    while not rospy.is_shutdown():
        # rate.sleep()
        rospy.sleep(0.01)
        screen.blit(img, (1,1))
        pygame.display.flip()
        drone_state = get_model_state('ardrone','world')
        rospy.loginfo("Robot position: x=%f, y=%f, z=%f" % (drone_state.pose.position.x, drone_state.pose.position.y, drone_state.pose.position.z))
        for event in pygame.event.get():
            if event.type == KEYDOWN: 
                # position control
                if event.key == pygame.K_UP:
                    print 'forward'
                    uav_state = 0
                    key_axes[0] = 1
                if event.key == pygame.K_DOWN:
                    print 'backward'
                    uav_state = 1
                    key_axes[1] = 1
                if event.key == pygame.K_LEFT:
                    print 'yaw left'
                    uav_state = 2
                    key_axes[2] = 1
                if event.key == pygame.K_RIGHT:
                    print 'yaw right'
                    uav_state = 3
                    key_axes[3] = 1
                # yaw and z control
                if event.key == pygame.K_w:
                    print 'up'
                    uav_state = 4
                    key_axes[4] = 1
                if event.key == pygame.K_s:
                    print 'down'
                    uav_state = 5
                    key_axes[5] = 1
                if event.key == pygame.K_a:
                    print 'turn left'
                    uav_state = 6
                    key_axes[6] = 1
                if event.key == pygame.K_d:
                    print 'turn right'
                    uav_state = 7
                    key_axes[7] = 1

            # when keyup, reset velcity
            elif event.type == pygame.KEYUP:
                uav_state = 8
                # # position control
                if event.key == pygame.K_UP:
                    # print 'forward'
                    key_axes[0] = 0
                if event.key == pygame.K_DOWN:
                    # print 'backward'
                    key_axes[1] = 0
                if event.key == pygame.K_LEFT:
                    # print 'left'
                    key_axes[2] = 0
                if event.key == pygame.K_RIGHT:
                    # print 'right'
                    key_axes[3] = 0
                # # yaw and z control
                if event.key == pygame.K_w:
                    # print 'up'
                    key_axes[4] = 0
                if event.key == pygame.K_s:
                    # print 'down'
                    key_axes[5] = 0
                if event.key == pygame.K_a:
                    # print 'turn left'
                    key_axes[6] = 0
                if event.key == pygame.K_d:
                    # print 'turn right'
                    key_axes[7] = 0

        print(key_axes) # watch key value

        if(key_axes[0]==1 and key_axes[1]==0):
            body_pose_msg.twist.linear.x = 1
            last_body_pose_msg.twist.linear.x = body_pose_msg.twist.linear.x
        if(key_axes[0]==0 and key_axes[1]==0):
            body_pose_msg.twist.linear.x = 0 
            last_body_pose_msg.twist.linear.x = body_pose_msg.twist.linear.x
        if(key_axes[1]==1 and key_axes[0]==0):
            body_pose_msg.twist.linear.x = -1
            last_body_pose_msg.twist.linear.x = body_pose_msg.twist.linear.x
        if(key_axes[1]==0 and key_axes[0]!=1):
            body_pose_msg.twist.linear.x = 0 
            last_body_pose_msg.twist.linear.x = body_pose_msg.twist.linear.x

        if(key_axes[2]==1 and key_axes[3]==0):
            body_pose_msg.twist.linear.y = 1
            last_body_pose_msg.twist.linear.y = body_pose_msg.twist.linear.y
        if(key_axes[2]==0 and key_axes[3]==0):
            body_pose_msg.twist.linear.y = 0
            last_body_pose_msg.twist.linear.y = body_pose_msg.twist.linear.y    
        if(key_axes[3]==1 and key_axes[2]==0):
            body_pose_msg.twist.linear.y = -1
            last_body_pose_msg.twist.linear.y = body_pose_msg.twist.linear.y
        if(key_axes[3]==0 and key_axes[2]!=1):
            body_pose_msg.twist.linear.y = 0
            last_body_pose_msg.twist.linear.y = body_pose_msg.twist.linear.y    

        if(key_axes[4]==1 and key_axes[5]==0):
            body_pose_msg.twist.linear.z = 1
            last_body_pose_msg.twist.linear.z = body_pose_msg.twist.linear.z
        if(key_axes[4]==0 and key_axes[5]==0):
            body_pose_msg.twist.linear.z = 0
            last_body_pose_msg.twist.linear.z = body_pose_msg.twist.linear.z  
        if(key_axes[5]==1 and key_axes[4]==0):
            body_pose_msg.twist.linear.z = -1
            last_body_pose_msg.twist.linear.z = body_pose_msg.twist.linear.z
        if(key_axes[5]==0 and key_axes[4]!=1):
            body_pose_msg.twist.linear.z = 0
            last_body_pose_msg.twist.linear.z = body_pose_msg.twist.linear.z  

        if(key_axes[6]==1 and key_axes[7]==0):
            body_pose_msg.twist.angular.z = 1
            last_body_pose_msg.twist.angular.z = body_pose_msg.twist.angular.z
        if(key_axes[6]==0 and key_axes[7]==0):
            body_pose_msg.twist.angular.z = 0
            last_body_pose_msg.twist.angular.z = body_pose_msg.twist.angular.z  
        if(key_axes[7]==1 and key_axes[6]==0):
            body_pose_msg.twist.angular.z = -1
            last_body_pose_msg.twist.angular.z = body_pose_msg.twist.angular.z
        if(key_axes[7]==0 and key_axes[6]!=1):
            body_pose_msg.twist.angular.z = 0
            last_body_pose_msg.twist.angular.z = body_pose_msg.twist.angular.z  
        
        # orientation ---> euler
        quaternion_uav = (drone_state.pose.orientation.w, drone_state.pose.orientation.x, drone_state.pose.orientation.y, drone_state.pose.orientation.z)
        euler_angles = tft.euler_from_quaternion(quaternion_uav, 'sxyz') # y
        if(euler_angles[0]>=0):
            uav_yaw = math.pi - euler_angles[0]
        if(euler_angles[0]<0):
            uav_yaw = -math.pi - euler_angles[0]
        print(uav_yaw)
        # body ---> world
        rotation_matrix_z = np.matrix([[math.cos(uav_yaw),-math.sin(uav_yaw),0],[math.sin(uav_yaw),math.cos(uav_yaw),0],[0,0,1]])
        rotation_matrix_z_inv = np.linalg.inv(rotation_matrix_z)
        body_vel_matrix = np.matrix([last_body_pose_msg.twist.linear.x, last_body_pose_msg.twist.linear.y, last_body_pose_msg.twist.linear.z])
        body_vel_matrix_trans = body_vel_matrix.T
        world_vel_matrix = rotation_matrix_z.dot(body_vel_matrix_trans)


        pub_pose_msg.pose.position.x = drone_state.pose.position.x
        pub_pose_msg.pose.position.y = drone_state.pose.position.y
        pub_pose_msg.pose.position.z = drone_state.pose.position.z

        pub_pose_msg.pose.orientation.w = drone_state.pose.orientation.w
        pub_pose_msg.pose.orientation.x = drone_state.pose.orientation.x
        pub_pose_msg.pose.orientation.y = drone_state.pose.orientation.y
        pub_pose_msg.pose.orientation.z = drone_state.pose.orientation.z

        pub_pose_msg.twist.linear.x = world_vel_matrix[0]
        pub_pose_msg.twist.linear.y = world_vel_matrix[1]
        pub_pose_msg.twist.linear.z = world_vel_matrix[2]
        # for i in range(100):
        #     pub_pose_msg.twist.linear.z +=0.01
        #     rospy.sleep(0.01)
        # for i in range(100):
        #     pub_pose_msg.twist.linear.z -=0.01
        #     rospy.sleep(0.01)

        pub_pose_msg.twist.angular.x = drone_state.twist.angular.x
        pub_pose_msg.twist.angular.y = drone_state.twist.angular.y
        pub_pose_msg.twist.angular.z = last_body_pose_msg.twist.angular.z

        key_pub.publish(pub_pose_msg)
    





if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

#######################################################
