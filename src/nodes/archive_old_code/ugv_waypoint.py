#
#!/usr/bin/env python


import sys
import itertools
import rospy
import math
#import random
from std_msgs.msg import Int64
from std_msgs.msg import Int64MultiArray
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
# from task_switch.msg import MRTAProblem
# from task_switch.msg import PMRTAResult

from allocation_common.msg import gazebo2world_info

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


#from mrta_msgs.msg import MRTAProblem

class UGV_Waypoint():

    def __init__(self):
        # ROS Initialize
        rospy.init_node('ugv_waypoint', anonymous=True)
        self.rate = rospy.Rate(10) # 10Hz
        print('Waypoint started')
        
        
        # Initialize Publisher & Subscriber
        
        # self.pub_pmrta_allocation_result = rospy.Publisher("/pmrta_allocation_result", PMRTAResult, queue_size=10)
        
        # self.update_mrta_problem = rospy.Subscriber("/mrta_problem_set", MRTAProblem, self.callback_problem)
        # self.update_robot_data = rospy.Subscriber("/robot_data_update", Int64, self.callback_robot)
        self.update_dynamics = rospy.Subscriber("/allocation_gazebo/gazebo2world_info", gazebo2world_info, self.callback_dynamics, queue_size=10)
        
        self.goal = MoveBaseGoal()

        


    def callback_dynamics(self, msg):
        # Subscriber to get the set of MRTA problem parameter published by mrta_problem_generator node
        print('Dynamics updated')
        # Update Problem Parameter
        for i in range(0,len(msg.gazebo_tasks_info)):
            task_id = msg.gazebo_task_info[i].task_id
            task_x = msg.gazebo_task_info[i].task_pose_x
            task_y = msg.gazebo_task_info[i].task_pose_y

        self.movebase_client(task_x,task_y)


    # def callback_problem(self, msg):
    #     # Subscriber to get the set of MRTA problem parameter published by mrta_problem_generator node

    #     # Update Problem Parameter
    #     self.robot_number = msg.robot_number
    #     self.robot_position_x = msg.robot_position_x
        
    

    # def callback_robot(self, msg):
    #     # subscriber to get status of mission success whether it is already finished or not
    #     self.robot_data = msg.data

    #     # Update Robot Position everytime new data is coming (pi' = pi)

    def movebase_client(self, target_x, target_y):

        # self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        # self.client.wait_for_server()

        # Define the goal
        # self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = 'jackal0/odom'
        self.goal.target_pose.pose.position.x = target_x
        self.goal.target_pose.pose.position.y = target_y
        self.goal.target_pose.pose.position.z = 0.0
        self.goal.target_pose.pose.orientation.x = 0
        self.goal.target_pose.pose.orientation.y = 0
        self.goal.target_pose.pose.orientation.z = 0
        self.goal.target_pose.pose.orientation.w = 1

        self.client.send_goal(goal)
        self.client.wait_for_result()
        if not self.wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            return self.client.get_result()

       
    def spin(self):
        while not rospy.is_shutdown():
            self.rate.sleep()


if __name__ == '__main__':
    try:
        ugv_waypoint = UGV_Waypoint()
        ugv_waypoint.spin()

    except rospy.ROSInterruptException:
        pass