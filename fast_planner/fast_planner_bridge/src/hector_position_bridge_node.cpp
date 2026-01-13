#include <ros/ros.h>
#include "fast_planner_bridge/hector_position_bridge.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "hector_position_bridge_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    ROS_INFO("Starting Hector Position Bridge Node...");
    ROS_INFO("This node converts Fast-Planner commands to Hector position controller format");

    fast_planner_bridge::HectorPositionBridge bridge(nh, nh_private);

    ros::spin();

    return 0;
}
