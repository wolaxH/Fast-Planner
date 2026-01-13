#include <ros/ros.h>
#include "fast_planner_bridge/gazebo_control_bridge.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "gazebo_control_bridge_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    ROS_INFO("Starting Gazebo Control Bridge Node...");

    fast_planner_bridge::GazeboControlBridge bridge(nh, nh_private);

    ros::spin();

    return 0;
}
