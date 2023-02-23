

// this header incorporates all the necessary #include files and defines the class "ExampleRosClass"
#include "mrta_uav_waypoint.h"

//CONSTRUCTOR:  this will get called whenever an instance of this class is created
// want to put all dirty work of initializations here
// odd syntax: have to pass nodehandle pointer into constructor for constructor to build subscribers, etc
UAV_Waypoint::UAV_Waypoint(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{ // constructor
    ROS_INFO("in class constructor of MRTA UAV Waypoint");
    initializeSubscribers(); // package up the messy work of creating subscribers; do this overhead in constructor
    initializePublishers();
    initializeServices();
    
    //initialize variables here, as needed
    k_ = -1;
    shadow_drift_x_ = -3;
    shadow_drift_y_ = 0.75;
    bebop_namespace = ros::this_node::getNamespace();
    ROS_INFO("Namespace Name : %s", bebop_namespace.c_str());
    bebop_id =  bebop_namespace.back() - '0';
    ROS_INFO("Bebop ID : %d", bebop_id);

    
    // can also do tests/waits to make sure all required services, topics, etc are alive
    
}

//member helper function to set up subscribers;
// note odd syntax: &ExampleRosClass::subscriberCallback is a pointer to a member function of ExampleRosClass
// "this" keyword is required, to refer to the current instance of ExampleRosClass
void UAV_Waypoint::initializeSubscribers()
{
    ROS_INFO("Initializing Subscribers");  
    dynamics_sub_ = nh_.subscribe("/allocation_gazebo/gazebo2world_info",10,&UAV_Waypoint::update_dynamics, this);
    waypoint_manual_sub_ = nh_.subscribe("/update_waypoint",10,&UAV_Waypoint::update_waypoint_manual, this);
    waypoint_allocation_sub_ = nh_.subscribe("/target_task_result",10,&UAV_Waypoint::update_waypoint_allocation, this);
   
    // add more subscribers here, as needed
}

//member helper function to set up services:
// similar syntax to subscriber, required for setting up services outside of "main()"
void UAV_Waypoint::initializeServices()
{
    ROS_INFO("Initializing Services");
    // add more services here, as needed
}

//member helper function to set up publishers;
void UAV_Waypoint::initializePublishers()
{
    ROS_INFO("Initializing Publishers");
    trajectory_pub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>(mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);
  
    //add more publishers, as needed
    // note: COULD make minimal_publisher_ a public member function, if want to use it within "main()"
}



// a simple callback function, used by the example subscriber.
// note, though, use of member variables and access to minimal_publisher_ (which is a member method)

void UAV_Waypoint::update_dynamics(const allocation_common::gazebo2world_info::ConstPtr& msg) {
    
    task_id_.clear();
    task_x_.clear();
    task_y_.clear();

    for(unsigned int i=0;i<msg->gazebo_tasks_info.size();i++)
    {
        task_id_.push_back(msg->gazebo_tasks_info[i].task_ID);
        task_x_.push_back(msg->gazebo_tasks_info[i].task_pose.x);
        task_y_.push_back(msg->gazebo_tasks_info[i].task_pose.y);
    }

    if (k_ >= 0) 
    {
        trajectory_msg_.header.stamp = ros::Time::now();
        Eigen::Vector3d desired_position(task_x_[k_]+shadow_drift_x_, task_y_[k_]+shadow_drift_y_, 1.5);
        // ROS_INFO("Go to Task %d", k_);
        desired_yaw_ = 0.0;
        mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position,desired_yaw_, &trajectory_msg_);

        trajectory_pub_.publish(trajectory_msg_);
    }
}

void UAV_Waypoint::update_waypoint_manual(const std_msgs::Int16::ConstPtr& msg) {

    k_ = msg->data;
}

void UAV_Waypoint::update_waypoint_allocation(const std_msgs::Int64MultiArray::ConstPtr& msg) {

    k_ = msg->data[bebop_id];
}




int main(int argc, char** argv) 
{
    // ROS set-ups:
    ros::init(argc, argv, "mrta_uav_waypoint"); //node name

    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor

    ROS_INFO("main: instantiating an object of type MRTA UAV Waypoint");
    UAV_Waypoint uav_waypoint(&nh);  //instantiate an ExampleRosClass object and pass in pointer to nodehandle for constructor to use

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
    
    ROS_INFO("main: going into spin; let the callbacks do all the work");
    
    ros::spin();
    return 0;
} 