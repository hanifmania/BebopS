#!/usr/bin/env python

import rospy
import math
import numpy as np
from numpy import random
from std_msgs.msg import Int64
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from task_switch.msg import MRTAProblem



class MRTAProblemGenerator():

    def __init__(self, isRandom=False):
        
         # ROS Initialize
        rospy.init_node('mrta_problem_generator', anonymous=True)
        self.rate = rospy.Rate(10) # 10Hz
        self.mission_status = False
        self.montecarlo_number = 100 # How many times to run the simulation
        self.montecarlo_id = 1 # Initialization of Montecarlo Simulation Counter

        # Initialize Variable to save MRTA Problem Parameter
        self.mrta_problem_message = MRTAProblem()
        
        # Initialize Publisher & Subscriber
        self.pub_mrta_problem = rospy.Publisher("/mrta_problem_set", MRTAProblem, queue_size=10)
        #self.pub_mrta_problem = rospy.Publisher("/mrta_problem_set", Int64, queue_size=10)
        
        self.is_mission_finished = rospy.Subscriber("/is_mission_finished", Bool, self.callback_montecarlo)
        #self.reset_service = rospy.Service("/reset_counter", SetBool, self.callback_reset_counter)

        # Load ROS Param (If want to use pre-defined problem (default setRandom = False)) 
        self.isRandom = isRandom # If the problem is generated randomly or not. Load from the YAML param if 'False' 
        
        if self.isRandom:
            #Randomize Value if True
            self.randomizeProblem() 
        else:
            # Load from param if False
            self.loadProblemParameter()

        rospy.sleep(1)

        self.mrta_problem_message.problem_id = self.montecarlo_id
        self.pub_mrta_problem.publish(self.mrta_problem_message)
        print('Running simulation number ', self.montecarlo_id)
        


    def callback_montecarlo(self, msg):
        # subscriber to get status of mission success whether it is already finished or not
        self.mission_status = msg.data

        if (self.mission_status == True):
            rospy.sleep(1)
            if (self.montecarlo_id < self.montecarlo_number):
                self.randomizeProblem()
                self.montecarlo_id += 1
                print('Running simulation number ', self.montecarlo_id)
                self.mrta_problem_message.problem_id = self.montecarlo_id
                self.pub_mrta_problem.publish(self.mrta_problem_message)



    def loadProblemParameter(self):
        # Procedure to load problem from ros param in config file and save it to msg format
        # self.robot_number = rospy.get_param("/robot_number")
        # self.robot_position_x = rospy.get_param("/robot_position/x")
        # self.robot_position_y = rospy.get_param("/robot_position/y")
        # self.robot_velocity = rospy.get_param("/robot_velocity")
        # self.robot_battery_level = rospy.get_param("/robot_battery_level")
        # self.robot_battery_discharge_rate = rospy.get_param("/robot_battery_discharge_rate")
        # self.robot_lambda = rospy.get_param("/robot_lambda")
        
        # self.task_number = rospy.get_param("/task_number")
        # self.task_position_x = rospy.get_param("/task_position/x")
        # self.task_position_y = rospy.get_param("/task_position/y")
        # self.task_velocity_x = rospy.get_param("/task_velocity/x")
        # self.task_velocity_y = rospy.get_param("/task_velocity/y")
        # self.task_priority = rospy.get_param("/task_priority")
        # self.task_operation_time = rospy.get_param("/task_operation_time")

        # self.time_horizon = rospy.get_param("/time_horizon")

        self.mrta_problem_message.robot_number = rospy.get_param("/robot_number")
        self.mrta_problem_message.robot_position_x = rospy.get_param("/robot_position/x")
        self.mrta_problem_message.robot_position_y = rospy.get_param("/robot_position/y")
        self.mrta_problem_message.robot_velocity = rospy.get_param("/robot_velocity")
        self.mrta_problem_message.robot_battery_level = rospy.get_param("/robot_battery_level")
        self.mrta_problem_message.robot_battery_discharge_rate = rospy.get_param("/robot_battery_discharge_rate")
        self.mrta_problem_message.robot_lambda = rospy.get_param("/robot_lambda")
        
        self.mrta_problem_message.task_number = rospy.get_param("/task_number")
        self.mrta_problem_message.task_position_x = rospy.get_param("/task_position/x")
        self.mrta_problem_message.task_position_y = rospy.get_param("/task_position/y")
        self.mrta_problem_message.task_velocity_x = rospy.get_param("/task_velocity/x")
        self.mrta_problem_message.task_velocity_y = rospy.get_param("/task_velocity/y")
        self.mrta_problem_message.task_priority = rospy.get_param("/task_priority")
        self.mrta_problem_message.task_operation_time = rospy.get_param("/task_operation_time")

        self.mrta_problem_message.time_horizon = rospy.get_param("/time_horizon")


    def randomizeProblem(self):
        # Procedure to randomize the value for the problem parameter and save it to msg format

        # Load Parameter for Randomization
        self.max_robot_number = rospy.get_param("/max_robot_number")
        self.max_task_number = rospy.get_param("/max_task_number")
        self.max_field_dimension_x = rospy.get_param("/max_field_dimension_x")
        self.max_field_dimension_y = rospy.get_param("/max_field_dimension_y")
        self.mrta_problem_message.time_horizon = rospy.get_param("/time_horizon")
        
        # Assign random value range for each parameter
        # random.randint(lower bound, higher bound, size) return integer value lower bound <= value <= higher bound for size mentioned
        # random.random(size) return open float value from [0,1)
        self.mrta_problem_message.robot_number = random.randint(1,self.max_robot_number)
        self.mrta_problem_message.robot_position_x = np.random.randint(0,self.max_field_dimension_x,size=self.mrta_problem_message.robot_number)
        self.mrta_problem_message.robot_position_y = np.random.randint(0,self.max_field_dimension_y,size=self.mrta_problem_message.robot_number)
        self.mrta_problem_message.robot_velocity = np.random.randint(4,20,size=self.mrta_problem_message.robot_number) # Set 4 m/s <= Robot Velocity <= 20 m/s
        self.mrta_problem_message.robot_battery_level = np.random.randint(0,100,size=self.mrta_problem_message.robot_number) # Set 0 battery unit <= Robot Battery Level <= 100 battery unit
        self.mrta_problem_message.robot_battery_discharge_rate = np.random.random(size=self.mrta_problem_message.robot_number) # Set robot battery discharge rate between 0 and 1 for every minute discharge rate
        self.mrta_problem_message.robot_lambda = np.random.randint(0,5,size=self.mrta_problem_message.robot_number) # Set lambda between 0 and 5. Higher value means higher penalty for robot use
        
        self.mrta_problem_message.task_number = random.randint(1,self.max_task_number)
        self.mrta_problem_message.task_position_x = np.random.randint(0,self.max_field_dimension_x,size=self.mrta_problem_message.task_number)
        self.mrta_problem_message.task_position_y = np.random.randint(0,self.max_field_dimension_y,size=self.mrta_problem_message.task_number)
        self.mrta_problem_message.task_velocity_x = np.random.randint(0,3,size=self.mrta_problem_message.task_number) # Set Max Task Velocity of 3 m/s in x dimension
        self.mrta_problem_message.task_velocity_y = np.random.randint(0,3,size=self.mrta_problem_message.task_number) # Set Max Task Velocity of 3 m/s in y dimension
        self.mrta_problem_message.task_priority = np.random.randint(0,10,size=self.mrta_problem_message.task_number) # Set Max Task Priority for 10. 
        self.mrta_problem_message.task_operation_time = np.random.randint(0,10,size=self.mrta_problem_message.task_number) # Max time operation time is 10 s to finish the task

        # self.mrta_problem_message.time_horizon = random.randint(4,self.max_task_number)
        

             
    
    def spin(self):
        while not rospy.is_shutdown():
            #rospy.loginfo("Problem Deployed")
            #self.pub_mrta_problem.publish(self.mrta_problem_message)
            self.rate.sleep()



if __name__ == '__main__':
    try:
        # problem = MRTAProblemGenerator(True)
        problem = MRTAProblemGenerator()
        problem.spin()

    except rospy.ROSInterruptException:
        pass