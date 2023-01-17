#!/usr/bin/env python
# license removed for brevity

import math
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionGoal
from allocation_common.msg import gazebo2world_info

# pub_pmrta_allocation_result = rospy.Publisher("/pmrta_allocation_result", PMRTAResult, queue_size=10)
k = 3
task_id = 0
task_x = 0
task_y = 0
# robot_id = 0
# robot_x = 0
# robot_y = 0
task_idl = [0,0,0,0,0,0]
task_xl = [0,0,0,0,0,0]
task_yl = [0,0,0,0,0,0]

def my_callback(event):
    global k
    k = k + 1

def callback_dynamics(msg):
        # Subscriber to get the set of MRTA problem parameter published by mrta_problem_generator node
        global task_id
        global task_x
        global task_y

        global task_idl
        global task_xl
        global task_yl
        # global robot_id
        # global robot_x
        # global robot_y

        # print('Dynamics updated')
        # Update Problem Parameter
        for i in range(0,len(msg.gazebo_tasks_info)):
            task_id = msg.gazebo_tasks_info[i].task_ID
            task_x = msg.gazebo_tasks_info[i].task_pose.x
            task_y = msg.gazebo_tasks_info[i].task_pose.y

        # for j in range(0,len(msg.gazebo_robots_info)):
        #     robot_id = msg.gazebo_robots_info[j].robot_ID
        #     robot_x = msg.gazebo_robots_info[j].robot_pose.position.x
        #     robot_y = msg.gazebo_robots_info[j].robot_pose.position.y

        for j in range(0,len(msg.gazebo_tasks_info)):
            task_idl[j] = msg.gazebo_tasks_info[j].task_ID
            task_xl[j] = msg.gazebo_tasks_info[j].task_pose.x
            task_yl[j] = msg.gazebo_tasks_info[j].task_pose.y

        # movebase_client(task_x,task_y)
        # result = movebase_client(task_x,task_y)
        # while (math.sqrt((task_x-robot_x)**2 + (task_y-robot_y)**2) >= 1):
        #     movebase_client(task_x-robot_x,task_y-robot_y)
        # rospy.sleep(1)

# def movebase_client():

#     client = actionlib.SimpleActionClient('jackal0/move_base',MoveBaseAction)
#     client.wait_for_server()

    
# # Define the goal
#     goal = MoveBaseGoal()
#     goal.target_pose.header.frame_id = "jackal0/odom"
#     goal.target_pose.pose.position.x = 0.0
#     goal.target_pose.pose.position.y = 0.0
#     goal.target_pose.pose.position.z = 0.0
#     goal.target_pose.pose.orientation.x = 0.0
#     goal.target_pose.pose.orientation.y = 0.0
#     goal.target_pose.pose.orientation.z = 0.0
#     goal.target_pose.pose.orientation.w = 1.0

#     client.send_goal(goal)
#     wait = client.wait_for_result()
#     if not wait:
#         rospy.logerr("Action server not available!")
#         rospy.signal_shutdown("Action server not available!")
#     else:
#         return client.get_result()

def movebase_client(target_x,target_y):

    client = actionlib.SimpleActionClient('jackal0/move_base',MoveBaseAction)
    client.wait_for_server()

    client.cancel_goal()
# Define the goal
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "jackal0/odom"
    goal.target_pose.pose.position.x = target_x - 1
    goal.target_pose.pose.position.y = target_y
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

# def movebase_client(target_x,target_y):

#     # client = actionlib.SimpleActionClient('jackal0/move_base',MoveBaseAction)
#     # client.wait_for_server()

#     pub_ugv_goal = rospy.Publisher("/jackal0/move_base/goal", MoveBaseActionGoal, queue_size=10)

    
# # Define the goal
#     ugv_goal = MoveBaseActionGoal()
#     ugv_goal.goal.target_pose.header.frame_id = "jackal0/odom"
#     ugv_goal.goal.target_pose.pose.position.x = target_x
#     ugv_goal.goal.target_pose.pose.position.y = target_y
#     ugv_goal.goal.target_pose.pose.position.z = 0.0
#     ugv_goal.goal.target_pose.pose.orientation.x = 0.0
#     ugv_goal.goal.target_pose.pose.orientation.y = 0.0
#     ugv_goal.goal.target_pose.pose.orientation.z = 0.0
#     ugv_goal.goal.target_pose.pose.orientation.w = 1.0

#     pub_ugv_goal.publish(ugv_goal)
#     print('Sending goal')

#     # client.send_goal(goal)
#     # wait = client.wait_for_result()
#     # if not wait:
#     #     rospy.logerr("Action server not available!")
#     #     rospy.signal_shutdown("Action server not available!")
#     # else:
#     #     return client.get_result()

if __name__ == '__main__':
    try:
        rospy.init_node('movebase_client_py')
        rate = rospy.Rate(10)
        print('Waypoint Started')
        update_dynamics = rospy.Subscriber("/allocation_gazebo/gazebo2world_info", gazebo2world_info, callback_dynamics, queue_size=10)
        rospy.Timer(rospy.Duration(60), my_callback)

        while not rospy.is_shutdown():
            #rospy.loginfo("Problem Deployed")
            # self.pub_pmrta_allocation_result.publish(self.allocation_result)
            print('Ride to Task ',k)
            result = movebase_client(task_xl[k]-3,task_yl[k]+0.75)
            rate.sleep()

        #result = movebase_client()
        # if result:
        #     rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        # rospy.loginfo("Navigation test finished.")
        pass