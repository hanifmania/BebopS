#!/usr/bin/env python
# license removed for brevity

import math
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionGoal
from allocation_common.msg import gazebo2world_info
from std_msgs.msg import Int16


class UGV_Waypoint():
    
    def __init__(self):
        # CONSTRUCTOR: this part will get called whenever an instance of this class is created
        # All of the initializations will be declared here
        
        # ROS Initialize
        rospy.init_node('ugv_waypoint', anonymous=True) # Initialize Node
        self.rate = rospy.Rate(10) # Set Node rate to 10Hz
        

        # Initialize Subscribers, Publishers, and Services
        self.initializeSubscribers()
        self.initializePublishers()
        self.initializeServices()

        # rospy.Timer(rospy.Duration(60), self.my_callback)

        # Initialize variables here
        self.jackal_namespace = rospy.get_namespace()[1:]    # Get namespace of the node and delete first character of it (To extract the namespace with id for move base frame)
        self.jackal_id = int(self.jackal_namespace[-2])      # Get Jackal ID from the namespace
        self.k = -1                                          # Waypoint target-k, k = -1 means that there is no active waypoint    
        self.task_id = []                                    # List to store active task-id 
        self.task_x = []                                     # List to store active task in x position
        self.task_y = []                                     # List to store active task in y position
        self.robot_id = []                                   # List to store active robot-id 
        self.robot_x = []                                    # List to store active robot in x position
        self.robot_y = []                                    # List to store active robot in y position
        self.isOriginSaved = False                           # Flag whether origin location of the robot has been saved or not
        self.shadow_drift_x = -3                             # The drift of the shadow location in the ground relative to the original task location (in the x-coordinate) 
        self.shadow_drift_y = 0.75                           # The drift of the shadow location in the ground relative to the original task location (in the y-coordinate) 
        self.origin_x = 0                                    # The default starting position (x-coordinate) of jackal-x (Will be updated by getOriginLocation() procedure
        self.origin_y = 0                                    # The default starting position (y-coordinate) of jackal-x (Will be updated by getOriginLocation() procedure
    
    def initializeSubscribers(self):
        # member helper function to set up subscribers (Put all of subscribers here)
        
        print('Initializing ROS Subscribers')
        # Subscriber for updating the dynamics position and velocity of each tasks and robots
        self.update_dynamics = rospy.Subscriber("/allocation_gazebo/gazebo2world_info", gazebo2world_info, self.callback_dynamics, queue_size=10)
        # Subscriber for updating which task as the target waypoint
        self.update_waypoint = rospy.Subscriber("/update_waypoint", Int16, self.callback_waypoint, queue_size=10)    
       
    
    def initializePublishers(self):
        # member helper function to set up publishers (Put all of publishers here)
        
        print('Initializing ROS Publishers')

    def initializeServices(self):
        # member helper function to set up services (Put all of services here)
        
        print('Initializing ROS Services')


    # def my_callback(self,event):
    #     self.k = self.k + 1

    def getOriginLocation(self):
        # Procedure to get origin location (starting deployment) in Gazebo for local navigation correction in move_base
        if (self.isOriginSaved == False):
            i = 0
            while (i<len(self.robot_id)) and (self.robot_id[i] != (5+self.jackal_id)):
                i = i + 1
            
            if (i<len(self.robot_id)):
                self.origin_x = self.robot_x[i]
                self.origin_y = self.robot_y[i]
                self.isOriginSaved = True



        # if (self.jackal_id < len(self.robot_id)):
        #     if (self.isOriginSaved == False):
        #         self.origin_x = self.robot_x[self.jackal_id]
        #         self.origin_y = self.robot_y[self.jackal_id]
        #         # print("Origin detected : ", self.origin_x," ---- ", self.origin_y)
        #         self.isOriginSaved = True

    def callback_dynamics(self,msg):
        # Subscriber to get the set of MRTA problem parameter published by mrta_problem_generator node

        # Update robots dynamics
        
        # Clear the previous states, and re-push the new states of robots
        self.robot_id.clear()
        self.robot_x.clear()
        self.robot_y.clear()

        # Update new states of robots
        for i in range(0,len(msg.gazebo_robots_info)):
            self.robot_id.append(msg.gazebo_robots_info[i].robot_ID)
            self.robot_x.append(msg.gazebo_robots_info[i].robot_pose.position.x)
            self.robot_y.append(msg.gazebo_robots_info[i].robot_pose.position.y)
        
        # Save origin location of robot (starting position)
        self.getOriginLocation()
        
        # Update tasks dynamics
        
        # Clear the previous states, and re-push the new states of robots
        self.task_id.clear()
        self.task_x.clear()
        self.task_y.clear()

        # Update new states of tasks
        for i in range(0,len(msg.gazebo_tasks_info)):
            self.task_id.append(msg.gazebo_tasks_info[i].task_ID)
            self.task_x.append(msg.gazebo_tasks_info[i].task_pose.x)
            self.task_y.append(msg.gazebo_tasks_info[i].task_pose.y)


    def callback_waypoint(self,msg):
        # Subscriber to get the set of MRTA problem parameter published by mrta_problem_generator node

        self.k = msg.data
        print('Receiving waypoint ',self.k)
        

    def movebase_client(self,target_x,target_y):

        client = actionlib.SimpleActionClient('move_base',MoveBaseAction) 
        client.wait_for_server()

        client.cancel_goal()
        
        # Define the goal
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = str(self.jackal_namespace) + "odom"
        goal.target_pose.pose.position.x = target_x - self.origin_x     
        goal.target_pose.pose.position.y = target_y - self.origin_y       
        goal.target_pose.pose.position.z = 0.0
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = 0.0
        goal.target_pose.pose.orientation.w = 1.0

        client.send_goal(goal)
        wait = client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            return client.get_result()
    
    def spin(self):
        while not rospy.is_shutdown():
            # If there is an active task and active waypoint then go to waypoint-k
            if(self.k >= 0):
                # print('Ride to Task ',self.k)
                if(self.task_id):
                    result = self.movebase_client(self.task_x[self.k]+self.shadow_drift_x,self.task_y[self.k]+self.shadow_drift_y)
            self.rate.sleep()



if __name__ == '__main__':
    try:   
        ugv_waypoint = UGV_Waypoint()
        ugv_waypoint.spin()
        
    except rospy.ROSInterruptException:
        pass