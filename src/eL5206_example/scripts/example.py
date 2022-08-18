#!/usr/bin/env python3

import rospy
import math
import sys
import std_msgs
import tf2_ros
import numpy
import geometry_msgs.msg
import nav_msgs.msg
from tf_conversions import transformations as t
import tf_conversions
import numpy as np

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose2D

class ExampleNode:
    def __init__(self):
        rospy.init_node('ExampleNode', anonymous=False)
        #Subscribers
        




        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)


        #Parameters
        self.update_rate    = 10.0
        self.robot_frame_id = 'base_footprint'
        self.odom_frame_id  = 'odom'



        self.currentScan =  LaserScan()
    

        

        # declare publisher of velocity so we can puclish a velocity command for robot
        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        
        # declare subscriber to laserscan  topic to get laser data
        rospy.Subscriber("scan", LaserScan, self.scan_callback)
        # subscriber to pose2d
        rospy.Subscriber("target_pose", Pose2D, self.pose_callback)
        
        self.update_timer       = rospy.Timer( rospy.Duration( 1.0 / self.update_rate ), self.timer_callback )

        pass

    def run(self):
        rospy.spin()
        pass



    def scan_callback(self, msg):
    
        self.currentScan = msg

    def pose_callback(self, msg):
      
        
        print('target pose is :')
        print(msg)
        print('starting to go there')
        # while we have not gotten there yet
        while True:
        
            move_cmd = Twist()
            move_cmd.linear.x = 0.10
            move_cmd.angular.z = 0.0  
            self.vel_pub.publish(move_cmd)   
                
        # stop when we get there (could also print pose)
        
        print('I am here , Stopping')
        move_cmd = Twist()
        move_cmd.linear.x = 0.0
        move_cmd.angular.z = 0.0  
        self.vel_pub.publish(move_cmd)  



    def timer_callback(self,event):
    
        try:
                trans = self.tfBuffer.lookup_transform(self.odom_frame_id, self.robot_frame_id , rospy.Time())
               
                
                
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                print("transformation not found doing nothing")
                return
                
        print(trans)
        # extract the information for Assignment 1 here   
                
                
        # uncomment the following 4 lines to move the robot         
        #move_cmd = Twist()
        #move_cmd.linear.x = 0.10
        #move_cmd.angular.z = 0.0    
        #self.vel_pub.publish(move_cmd)         
                
        


        
        
        
        pass

if __name__ == '__main__':
    node = ExampleNode()
    node.run()
