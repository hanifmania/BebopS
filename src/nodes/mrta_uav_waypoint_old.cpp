/*
 * Copyright 2018 Giuseppe Silano, University of Sannio in Benevento, Italy
 * Copyright 2018 Pasquale Oppido, University of Sannio in Benevento, Italy
 * Copyright 2018 Luigi Iannelli, University of Sannio in Benevento, Italy
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <thread>
#include <chrono>

#include <Eigen/Core>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <allocation_common/gazebo2world_info.h>
#include <std_msgs/Int16.h>

const int TASK_MAX = 10;
// int k = -2;
int k = -1;
int task_id;
double task_x;
double task_y;

int task_idl [TASK_MAX];
double task_xl [TASK_MAX];
double task_yl [TASK_MAX];
ros::Publisher trajectory_pub;


// void waypoint_callback(const ros::TimerEvent& event)
// {
//   k = k + 1;
  
// }


void update_waypoint(const std_msgs::Int16::ConstPtr& msg)
{
    // Code to handle received message goes here
    k = msg->data;
}

void update_dynamics(const allocation_common::gazebo2world_info::ConstPtr & msg) 
{
  // for(unsigned int j=0;j<msg->gazebo_tasks_info.size();j++)
  // {
  //     task_id=msg->gazebo_tasks_info[j].task_ID;
  //     task_x=msg->gazebo_tasks_info[j].task_pose.x;
  //     task_y=msg->gazebo_tasks_info[j].task_pose.y;
  // }

  for(unsigned int j=0;j<msg->gazebo_tasks_info.size();j++)
  {
      task_idl[j]=msg->gazebo_tasks_info[j].task_ID;
      task_xl[j]=msg->gazebo_tasks_info[j].task_pose.x;
      task_yl[j]=msg->gazebo_tasks_info[j].task_pose.y;
  }


  if (k >= 0) {
      trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
      trajectory_msg.header.stamp = ros::Time::now();
      Eigen::Vector3d desired_position(task_xl[k]-2.75, task_yl[k]+0.75, 1.5);
      ROS_INFO("Go to Task %d", k);
      double desired_yaw = 0.0;
      mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position,
          desired_yaw, &trajectory_msg);

      // ROS_INFO("Publishing waypoint on namespace: [%f, %f, %f].",
      //          desired_position.x(),
      //          desired_position.y(),
      //          desired_position.z());
      trajectory_pub.publish(trajectory_msg); 
  }
   
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "mrta_uav_waypoint");
  ros::NodeHandle nh;
  trajectory_pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);
  ros::Subscriber dynamics_sub = nh.subscribe("/allocation_gazebo/gazebo2world_info",10,update_dynamics);
  ros::Subscriber waypoint_sub = nh.subscribe<std_msgs::Int16>("/update_waypoint",10,update_waypoint);
  
  // ros::Timer waypoint_timer = nh.createTimer(ros::Duration(60), waypoint_callback);

  ROS_INFO("Started UAV Waypoint Node");

  std_srvs::Empty srv;
  bool unpaused = ros::service::call("/gazebo/unpause_physics", srv);
  unsigned int i = 0;

  // Trying to unpause Gazebo for 10 seconds.
  while (i <= 10 && !unpaused) {
    ROS_INFO("Wait for 1 second before trying to unpause Gazebo again.");
    std::this_thread::sleep_for(std::chrono::seconds(1));
    unpaused = ros::service::call("/gazebo/unpause_physics", srv);
    ++i;
  }

  if (!unpaused) {
    ROS_FATAL("Could not wake up Gazebo.");
    return -1;
  }
  else {
    ROS_INFO("Unpaused the Gazebo simulation.");
  }

  // Wait for t seconds to let the Gazebo GUI show up.
  // double t = 0.5;
  // ros::Duration(t).sleep();

  // trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
  // trajectory_msg.header.stamp = ros::Time::now();
  // Eigen::Vector3d desired_position(0.0, 0.0, 2.0);
  // double desired_yaw = 0.0;
  // mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position,
  //     desired_yaw, &trajectory_msg);

  // ROS_INFO("Publishing waypoint on namespace %s: [%f, %f, %f].",
  //          nh.getNamespace().c_str(),
  //          desired_position.x(),
  //          desired_position.y(),
  //          desired_position.z());
  // trajectory_pub.publish(trajectory_msg);

  ros::spin();
}
