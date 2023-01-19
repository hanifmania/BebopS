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

class Mission_Manager():

    def __init__(self):
        # CONSTRUCTOR: this part will get called whenever an instance of this class is created
        # All of the initializations will be declared here
        
        # ROS Initialize
        rospy.init_node('mission_manager', anonymous=True) # Initialize Node
        self.rate = rospy.Rate(10) # Set Node rate to 10Hz
        print("Mission Manager Node is working here")

        # Initialize Subscribers, Publishers, and Services
        self.initializeSubscribers()
        self.initializePublishers()
        self.initializeServices()

        rospy.Timer(rospy.Duration(30), self.my_callback)

        # Initialize variables here
        self.robot_id = []
        self.robot_pose_x = []
        self.robot_pose_y = []
        self.task_id = []
        self.task_pose_x = []
        self.task_pose_y = []
        self.task_vel_x = []
        self.task_vel_y = []

        self.target_waypoint_mpc = []

        self.mrta_problem_message = MRTAProblem()
        self.mrta_problem_message.robot_velocity = []
        self.mrta_problem_message.robot_battery_level = []
        self.mrta_problem_message.robot_battery_discharge_rate = []
        self.mrta_problem_message.robot_lambda = []

        self.target_task = Int64MultiArray()
        self.target_task.data = [-1] * 10
        # print("Target Task : ", self.target_task.data)

        # Sending First MRTA Problem to the Allocator
        print("Sending first problem")
        rospy.sleep(1)                                  # Wait for the callback_dynamics to save the robot and task states 
        self.publish_mrta_problem()


        
       

    
    def initializeSubscribers(self):
        # member helper function to set up subscribers (Put all of subscribers here)
        
        print('Initializing ROS Subscribers')
        self.update_dynamics = rospy.Subscriber("/allocation_gazebo/gazebo2world_info", gazebo2world_info, self.callback_dynamics)
        self.update_pmrta_allocation = rospy.Subscriber("/pmrta_allocation_result", PMRTAResult, self.callback_pmrta)


    def initializePublishers(self):
        # member helper function to set up publishers (Put all of publishers here)
        
        print('Initializing ROS Publishers')
        self.pub_mrta_problem = rospy.Publisher("/mrta_problem_set", MRTAProblem, queue_size=10)
        self.pub_target_task = rospy.Publisher("/target_task_result", Int64MultiArray, queue_size=10)
        
        
    def initializeServices(self):
        # member helper function to set up services (Put all of services here)
        
        print('Initializing ROS Services')
        

    def my_callback(self,event):
        print("Publishing next problem")
        self.publish_mrta_problem() 
    
    def callback_dynamics(self, msg):
        # Callback for update_dynamics subscriber : Update All of Robots and Tasks information from Gazebo Dynamics
    
        # Update robots dynamics
        # print("Updating Dynamics is running")
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
        
        # print("This is Callback No : ", self.cb_num)
        # print("Robot ID : ", self.robot_id)
        # print("Task ID : ", self.task_id)
        # self.cb_num = self.cb_num + 1

        # if (self.isProblemSent == False):
        #     self.publish_mrta_problem()
        #     self.isProblemSent = True
        

    
    def callback_pmrta(self, msg):
        # subscriber to get status of mission success whether it is already finished or not
        
        print("Welcome to the PMRTA Result")
        self.pmrta_allocation_result = msg.allocation_result
        self.robot_number = msg.robot_number
        self.task_number = msg.task_number
        self.time_horizon = msg.time_horizon
        
        self.allocation_result = np.zeros([self.robot_number,self.task_number,self.time_horizon],dtype=float)
        self.target_waypoint = -1*np.ones([self.robot_number,self.time_horizon],dtype=int)#,dtype= int)

        count = 0

        for i in range(0,self.robot_number):
            for j in range(0, self.task_number):
                for k in range(0,self.time_horizon):
                    self.allocation_result[i,j,k] = self.pmrta_allocation_result[count]
                    count = count + 1
                    if (self.allocation_result[i,j,k] == 1):
                        self.target_waypoint[i,k] = j#+1
         
        print(self.target_waypoint)
        
        self.target_waypoint_mpc = self.target_waypoint[:,0]
        # print(self.target_waypoint[:,0])
        print("Sequence of Waypoint : ", self.target_waypoint_mpc)
        
        
        self.publish_target_task()

        # rospy.sleep(1)
        # self.publish_mrta_problem()
        # print('Publishing new problem')

    def publish_mrta_problem(self):
        # Procedure to publish mrta problem set to be allocated
        
        self.mrta_problem_message.robot_number = len(self.robot_id)
        self.mrta_problem_message.robot_position_x = self.robot_pose_x
        self.mrta_problem_message.robot_position_y = self.robot_pose_y
        
        self.mrta_problem_message.robot_velocity.clear()
        self.mrta_problem_message.robot_battery_level.clear()
        self.mrta_problem_message.robot_battery_discharge_rate.clear()
        self.mrta_problem_message.robot_lambda.clear()

        for i in range(0,len(self.robot_id)):
            if(self.robot_id[i] <= 4):
                self.mrta_problem_message.robot_velocity.append(8.0)
                self.mrta_problem_message.robot_battery_level.append(100.0)
                self.mrta_problem_message.robot_battery_discharge_rate.append(0.5)
                self.mrta_problem_message.robot_lambda.append(5)
            else:
                self.mrta_problem_message.robot_velocity.append(2.0)
                self.mrta_problem_message.robot_battery_level.append(100.0)
                self.mrta_problem_message.robot_battery_discharge_rate.append(0.5)
                self.mrta_problem_message.robot_lambda.append(5)
         
        
        self.mrta_problem_message.task_number = len(self.task_id)
        self.mrta_problem_message.task_position_x = self.task_pose_x
        self.mrta_problem_message.task_position_y = self.task_pose_y
        self.mrta_problem_message.task_velocity_x = self.task_vel_x
        self.mrta_problem_message.task_velocity_y = self.task_vel_y
        self.mrta_problem_message.task_priority = [1] * len(self.task_id)
        self.mrta_problem_message.task_operation_time = [10] * len(self.task_id)

        self.mrta_problem_message.time_horizon = 4        
        
        self.pub_mrta_problem.publish(self.mrta_problem_message)

        # print('mrta problem is published')

    def publish_target_task(self):
        # Procedure to publish target task result to each robots    
        
        for i in range(0,len(self.robot_id)):
            self.target_task.data[self.robot_id[i]] = self.target_waypoint_mpc[i]
        
        print("Finalized Waypoint Sequence : ", self.target_task.data)
        self.pub_target_task.publish(self.target_task)

        
    
    
   

    # def compute_interception(self,p_i,q_j,vq_j,vp_i):
    #     # p_i is a robot position in one-dimensional vector of 2 values (x,y)
    #     # q_j is a task position in one-dimensional vector of 2 values (x,y)
    #     # vq_j is a task velocity in one-dimensional vector of 2 values (vx,vy)
    #     # v_i is a robot velocity in one-dimensional vector of 2 values (vx,vy)
       
    #     delta_x = q_j[0] - p_i[0]
    #     delta_y = q_j[1] - p_i[1]

    #     a = vq_j[0]**2 + vq_j[1]**2 - vp_i**2
    #     b = 2*(vq_j[0]*delta_x + vq_j[1]*delta_y)
    #     c = delta_x**2 + delta_y**2
    #     D = b**2 - 4*a*c

    #     t1 = (-b + math.sqrt(D))/(2*a)
    #     t2 = (-b - math.sqrt(D))/(2*a)

    #     if (D >= 0):
    #         if (t1*t2 <= 0):
    #             t = max(t1,t2)
    #         else:
    #             if(t1>0 and t2>0):
    #                 t = min(t1,t2)
    #             else:
    #                 t = inf
    #     else:
    #         t = inf
        
 
    #     result_interception = q_j + vq_j*t
        
    #     return result_interception    
    
    def spin(self):
        while not rospy.is_shutdown():
            # self.publish_mrta_problem()
            # self.publish_mrta_problem()
            self.rate.sleep()



if __name__ == '__main__':
    try:
        mission_manager = Mission_Manager()
        mission_manager.spin()
        

    except rospy.ROSInterruptException:
        pass