#!/usr/bin/env python

import rospy
import math
import numpy as np
from numpy import random
#import random
from std_msgs.msg import Int64
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from task_switch.msg import MRTAProblem
#from mrta_msgs.msg import MRTAProblem

class Genetic_Allocator():

    def __init__(self):
        # ROS Initialize
        rospy.init_node('genetic_allocator', anonymous=True)
        self.rate = rospy.Rate(10) # 10Hz
        self.allocation_result = 10

        # Initialize Publisher & Subscriber
        #self.pub_mrta_problem = rospy.Publisher("/mrta_problem_generator", MRTAProblem, queue_size=10)
        self.pub_genetic_allocation_result = rospy.Publisher("/genetic_allocation_result", Int64, queue_size=10)
        
        
        self.update_mrta_problem = rospy.Subscriber("/mrta_problem_set", MRTAProblem, self.callback_problem)
        self.update_robot_data = rospy.Subscriber("/robot_data_update", Int64, self.callback_robot)
        self.update_task_data = rospy.Subscriber("/task_data_update", Int64, self.callback_task)
        self.update_mission_status = rospy.Subscriber("/is_mission_finished", Bool, self.callback_mission_status)
        
        #self.reset_service = rospy.Service("/reset_counter", SetBool, self.callback_reset_counter)


    def callback_problem(self, msg):
        # subscriber to get status of mission success whether it is already finished or not
        self.problem_set = msg.robot_number

    def callback_robot(self, msg):
        # subscriber to get status of mission success whether it is already finished or not
        self.robot_data = msg.data

    def callback_task(self, msg):
        # subscriber to get status of mission success whether it is already finished or not
        self.task_data = msg.data


    def callback_mission_status(self, msg):
        # subscriber to get status of mission success whether it is already finished or not
        self.mission_status = msg.data
        self.allocation_result = self.mission_status

        
    
    def spin(self):
        while not rospy.is_shutdown():
            #rospy.loginfo("Problem Deployed")
            self.pub_genetic_allocation_result.publish(self.allocation_result)
            self.rate.sleep()



if __name__ == '__main__':
    try:
        genetic_allocator = Genetic_Allocator()
        genetic_allocator.spin()

    except rospy.ROSInterruptException:
        pass