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

class Mission_Data_Center():

    def __init__(self):
        # ROS Initialize
        rospy.init_node('mission_data_center', anonymous=True)
        self.rate = rospy.Rate(10) # 10Hz
        #self.allocation_result = 10

        # Initialize Publisher & Subscriber
        #self.pub_mrta_problem = rospy.Publisher("/mrta_problem_generator", MRTAProblem, queue_size=10)
        self.pub_robot_data = rospy.Publisher("/robot_data_update", Int64, queue_size=10)
        self.pub_task_data = rospy.Publisher("/task_data_update", Int64, queue_size=10)
        self.pub_mission_status = rospy.Publisher("/is_mission_finished", Bool, queue_size=10)
        self.pub_data_record = rospy.Publisher("/data_record", Int64, queue_size=10)
        
        self.update_pmrta_allocation = rospy.Subscriber("/pmrta_allocation_result", PMRTAResult, self.callback_pmrta)
        self.update_genetic_allocation = rospy.Subscriber("/genetic_allocation_result", Int64, self.callback_genetic)
       
        # self.update_robot_dynamics = rospy.Subscriber("/mrta_problem_set", Int64, self.callback_problem)
        # self.update_robot_data = rospy.Subscriber("/robot_data_update", Int64, self.callback_robot)
        self.update_task_data = rospy.Subscriber("/task_dynamics_update", Int64, self.callback_task)
        self.update_battery_data = rospy.Subscriber("/battery_dynamics_update", Int64, self.callback_battery)
        self.update_jackal_0 = rospy.Subscriber("/jackal0/joint_states", JointState, self.callback_jackal_0)
        self.update_jackal_1 = rospy.Subscriber("/jackal1/joint_states", JointState, self.callback_jackal_1)
        self.update_bebop_0 = rospy.Subscriber("/bebop0/odometry", Odometry, self.callback_bebop_0)
        self.update_bebop_1 = rospy.Subscriber("/bebop1/odometry", Odometry, self.callback_bebop_1)
        
        # self.update_mission_status = rospy.Subscriber("/is_mission_finished", Bool, self.callback_mission_status)
        
        #self.reset_service = rospy.Service("/reset_counter", SetBool, self.callback_reset_counter)

    def callback_bebop_0(self, msg):
        # subscriber to get status of mission success whether it is already finished or not
        # self.bebop0_odom = msg.data
        self.bebop0_odom = 10


    def callback_bebop_1(self, msg):
        # subscriber to get status of mission success whether it is already finished or not
        # self.bebop1_odom = msg.data
        self.bebop1_odom = 10

    def callback_jackal_0(self, msg):
        # subscriber to get status of mission success whether it is already finished or not
        # self.jackal0_odom = msg.data
        self.jackal0_odom = 10

    def callback_jackal_1(self, msg):
        # subscriber to get status of mission success whether it is already finished or not
        # self.jackal1_odom = msg.data
        self.jackal1_odom = 10

    def callback_pmrta(self, msg):
        # subscriber to get status of mission success whether it is already finished or not
        self.pmrta_allocation_result = msg.allocation_result
        self.robot_number = msg.robot_number
        self.task_number = msg.task_number
        self.time_horizon = msg.time_horizon
        
        self.allocation_result = np.zeros([self.robot_number,self.task_number,self.time_horizon],dtype=float)
        self.target_waypoint = np.zeros([self.robot_number,self.time_horizon],dtype=int)#,dtype= int)

        count = 0

        for i in range(0,self.robot_number):
            for j in range(0, self.task_number):
                for k in range(0,self.time_horizon):
                    self.allocation_result[i,j,k] = self.pmrta_allocation_result[count]
                    count = count + 1
                    if (self.allocation_result[i,j,k] == 1):
                        self.target_waypoint[i,k] = j+1
         
        print(self.target_waypoint)
        
        #############################################################################################################
        # Publish Waypoint Procedure
            # Update Robot Position, Robot Velcoity, Task Position, Task Velocity
            # Compute Intersection, Time INtersection, and Vector velocity of Robot Speed of target waypoint matrix
            # Send And Publish Waypoint (Pose : Intersection Point Twist : Vector Velocity)
        #############################################################################################################

        rospy.sleep(1)
        self.pub_mission_status.publish(True)

    def callback_genetic(self, msg):
        # subscriber to get status of mission success whether it is already finished or not
        self.genetic_result = msg.data

    # def callback_problem(self, msg):
    #     # subscriber to get status of mission success whether it is already finished or not
    #     self.problem_set = msg.data

    # def callback_robot(self, msg):
    #     # subscriber to get status of mission success whether it is already finished or not
    #     self.robot_data = msg.data

    def callback_task(self, msg):
        # subscriber to get status of mission success whether it is already finished or not
        self.task_data = msg.data
        #############################################################################################################
        # Update Task Position
        #############################################################################################################

    def callback_battery(self, msg):
        # subscriber to get status of mission success whether it is already finished or not
        self.battery_data = msg.data
        #############################################################################################################
        # Update Battery Status
        #############################################################################################################

    # def callback_mission_status(self, msg):
    #     # subscriber to get status of mission success whether it is already finished or not
    #     self.mission_status = msg.data
    #     self.allocation_result = self.mission_status

    def compute_interception(self,p_i,q_j,vq_j,vp_i):
        # p_i is a robot position in one-dimensional vector of 2 values (x,y)
        # q_j is a task position in one-dimensional vector of 2 values (x,y)
        # vq_j is a task velocity in one-dimensional vector of 2 values (vx,vy)
        # v_i is a robot velocity in one-dimensional vector of 2 values (vx,vy)
       
        delta_x = q_j[0] - p_i[0]
        delta_y = q_j[1] - p_i[1]

        a = vq_j[0]**2 + vq_j[1]**2 - vp_i**2
        b = 2*(vq_j[0]*delta_x + vq_j[1]*delta_y)
        c = delta_x**2 + delta_y**2
        D = b**2 - 4*a*c

        t1 = (-b + math.sqrt(D))/(2*a)
        t2 = (-b - math.sqrt(D))/(2*a)

        if (D >= 0):
            if (t1*t2 <= 0):
                t = max(t1,t2)
            else:
                if(t1>0 and t2>0):
                    t = min(t1,t2)
                else:
                    t = inf
        else:
            t = inf
        
 
        result_interception = q_j + vq_j*t
        
        return result_interception    
    
    def spin(self):
        while not rospy.is_shutdown():
            #rospy.loginfo("Problem Deployed")
            #self.pub_robot_data.publish(self.allocation_result)
            #self.pub_task_data.publish(self.allocation_result)
            # self.pub_mission_status.publish(False)
            #self.pub_data_record.publish(self.allocation_result)

            self.rate.sleep()



if __name__ == '__main__':
    try:
        mission_data_center = Mission_Data_Center()
        mission_data_center.spin()

    except rospy.ROSInterruptException:
        pass