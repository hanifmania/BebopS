#!/usr/bin/env python

import rospy
import math
import numpy as np
from numpy import random
#import random
from std_msgs.msg import Int64
from std_msgs.msg import Int64MultiArray
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool

from task_switch.msg import PMRTAResult
from task_switch.msg import MRTAProblem
from allocation_common.msg import gazebo2world_info

class Task_Dynamics():

    def __init__(self):
        # CONSTRUCTOR: this part will get called whenever an instance of this class is created
        # All of the initializations will be declared here
        
        # ROS Initialize
        rospy.init_node('task_dynamics', anonymous=True) # Initialize Node
        self.rate = rospy.Rate(10) # Set Node rate to 10Hz
        

        # Initialize Subscribers, Publishers, and Services
        self.initializeSubscribers()
        self.initializePublishers()
        self.initializeServices()


        # Initialize variables here
        self.waypoint_tolerance = 1
        self.target_task = Int64MultiArray()
        self.target_task.data = [-1] * 10

        self.robot_id = []
        self.robot_pose_x = []
        self.robot_pose_y = []
        self.task_id = []
        self.task_pose_x = []
        self.task_pose_y = []
        self.task_vel_x = []
        self.task_vel_y = []
        self.task_operation_time = []
        self.task_status = []
        self.shadow_drift_x = -3                             # The drift of the shadow location in the ground relative to the original task location (in the x-coordinate) 
        self.shadow_drift_y = 0.75                           # The drift of the shadow location in the ground relative to the original task location (in the y-coordinate) 
   

        #Initialize algorithm here
        rospy.sleep(1)                                       # Wait for the topics callback to be received
        self.initializeTaskStatus()                          # Initialize Task Operation Time and Task Status Dynamics

    
    def initializeSubscribers(self):
        # member helper function to set up subscribers (Put all of subscribers here)
        
        print('Initializing ROS Subscribers')
        self.update_dynamics = rospy.Subscriber("/allocation_gazebo/gazebo2world_info", gazebo2world_info, self.callback_dynamics)
        self.update_target_task = rospy.Subscriber("/target_task_result", Int64MultiArray, self.callback_waypoint_sequence)


    def initializePublishers(self):
        # member helper function to set up publishers (Put all of publishers here)
        
        print('Initializing ROS Publishers')
        self.pub_event_trigger = rospy.Publisher("/task_update_allocation", Int64MultiArray, queue_size=10)
        
        
    def initializeServices(self):
        # member helper function to set up services (Put all of services here)
        
        print('Initializing ROS Services')
        
        
    def initializeTaskStatus(self):
        # member helper function to initialize task operation time and task status in the beginning of this node
        
        self.task_operation_time.clear()
        self.task_status.clear()

        print('Initializing Task Status')
        for i in range(0,len(self.task_id)):
            self.task_operation_time.append(5)  # Assume all of them is 5 seconds. Can be randomized next
            self.task_status.append(1)          # 1 means task is active, while 0 means task is inactive
        
        print("Task ID : ", self.task_id)
        print("Task Operation Time ", self.task_operation_time)
        print("Task Status ", self.task_status)

    
    def euclidian_distance(self,a,b,c,d):
        # member helper function to calculate euclidian distance between point (a,b) and point (c,d) in the 2-dimensional cartesian coordinates
        return math.sqrt((a-c)**2+(b-d)**2)


    def evaluateTaskStatus(self):
        # member helper function to check the task status and task operation time while mission is being performed
        print("Evaluating Task Status")
        print("Task Operation Time ", self.task_operation_time)
        print("Task Status ", self.task_status)

        # Task status update : when task operation time is decreasing into 0, then the task is assumed to be done and thus task status become 0
        for i in range(0,len(self.task_operation_time)):
            if(self.task_operation_time[i]<= 0):
                self.task_status[i] = 0
            
        # Task operation time update : If the robot is being allocated to the task and is inside the waypoint tolerance boundary within the task, then the task operation time will decrease by time
        for i in range(0,10):
            if(self.target_task.data[i]>=0):
                if(self.task_status[self.target_task.data[i]] == 1):
                    if(self.euclidian_distance(self.task_pose_x[self.target_task.data[i]]+self.shadow_drift_x,self.task_pose_y[self.target_task.data[i]]+self.shadow_drift_y,self.robot_pose_x[i-5],self.robot_pose_y[i-5]) <= self.waypoint_tolerance):
                        self.task_operation_time[self.target_task.data[i]] = self.task_operation_time[self.target_task.data[i]] - 0.05

    

    def callback_waypoint_sequence(self,msg):
        # Subscriber to get the set of MRTA problem parameter published by mrta_problem_generator node

        self.target_task.data = msg.data
    
    
    
    def callback_dynamics(self, msg):
        # Callback for update_dynamics subscriber : Update All of Robots and Tasks information from Gazebo Dynamics
    
        # Update robots dynamics
        
        # Clear the previous states, and re-push the new states of robots
        
        self.robot_id.clear()
        self.robot_pose_x.clear()
        self.robot_pose_y.clear()

        # Update new states of robots
        for i in range(0,len(msg.gazebo_robots_info)):
            self.robot_id.append(msg.gazebo_robots_info[i].robot_ID)
            self.robot_pose_x.append(msg.gazebo_robots_info[i].robot_pose.position.x)
            self.robot_pose_y.append(msg.gazebo_robots_info[i].robot_pose.position.y)
        
        
        # Update tasks dynamics
        
        # Clear the previous states, and re-push the new states of robots
        self.task_id.clear()
        self.task_pose_x.clear()
        self.task_pose_y.clear()
        self.task_vel_x.clear()
        self.task_vel_y.clear()
        
        # Update new states of tasks
        for i in range(0,len(msg.gazebo_tasks_info)):
            self.task_id.append(msg.gazebo_tasks_info[i].task_ID)
            self.task_pose_x.append(msg.gazebo_tasks_info[i].task_pose.x)
            self.task_pose_y.append(msg.gazebo_tasks_info[i].task_pose.y)
            self.task_vel_x.append(msg.gazebo_tasks_info[i].task_twist.x)
            self.task_vel_y.append(msg.gazebo_tasks_info[i].task_twist.y)
        
        
        
    
    def spin(self):
        while not rospy.is_shutdown():
            
            self.evaluateTaskStatus()
            self.rate.sleep()



if __name__ == '__main__':
    try:
        task_dynamics = Task_Dynamics()
        task_dynamics.spin()
        

    except rospy.ROSInterruptException:
        pass