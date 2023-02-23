

// here's a good trick--should always do this with header files:
// create a unique mnemonic for this header file, so it will get included if needed,
// but will not get included multiple times
#ifndef MRTA_UAV_WAYPOINT_H_
#define MRTA_UAV_WAYPOINT_H_

//some generically useful stuff to include...
#include <thread>
#include <chrono>

#include <Eigen/Core>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>


#include <ros/ros.h> 

//message types used in this example code;  include more message types, as needed
#include <std_msgs/Bool.h> 
#include <std_msgs/Float32.h>
#include <std_srvs/Empty.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <allocation_common/gazebo2world_info.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int64MultiArray.h>

// define a class, including a constructor, member variables and member functions
class UAV_Waypoint
{
public:
    UAV_Waypoint(ros::NodeHandle* nodehandle); //"main" will need to instantiate a ROS nodehandle, then pass it to the constructor
    // may choose to define public methods or public variables, if desired
private:
    // put private member data here;  "private" data will only be available to member functions of this class;
    ros::NodeHandle nh_; // we will need this, to pass between "main" and constructor
    // some objects to support subscriber, service, and publisher
     
    ros::Subscriber dynamics_sub_;
    ros::Subscriber waypoint_manual_sub_;
    ros::Subscriber waypoint_allocation_sub_;
    ros::Publisher trajectory_pub_;

    int k_;
    std::vector<int> task_id_;
    std::vector<double> task_x_;
    std::vector<double> task_y_;

    double desired_yaw_;
    double shadow_drift_x_;
    double shadow_drift_y_;
    std::string bebop_namespace;
    int bebop_id;

    trajectory_msgs::MultiDOFJointTrajectory trajectory_msg_;

    // member methods as well:
    void initializeSubscribers(); // we will define some helper methods to encapsulate the gory details of initializing subscribers, publishers and services
    void initializePublishers();
    void initializeServices();
    
    void update_dynamics(const allocation_common::gazebo2world_info::ConstPtr& msg); 
    void update_waypoint_manual(const std_msgs::Int16::ConstPtr& msg);
    void update_waypoint_allocation(const std_msgs::Int64MultiArray::ConstPtr& msg);
}; // note: a class definition requires a semicolon at the end of the definition

#endif  // this closes the header-include trick...ALWAYS need one of these to match #ifndef