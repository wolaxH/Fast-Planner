#ifndef GAZEBO_CONTROL_BRIDGE_H
#define GAZEBO_CONTROL_BRIDGE_H

#include <ros/ros.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/ModelState.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <cmath>

namespace fast_planner_bridge {

class GazeboControlBridge {
public:
    GazeboControlBridge(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
    ~GazeboControlBridge() = default;

private:
    // Callback functions
    void positionCommandCallback(const quadrotor_msgs::PositionCommand::ConstPtr& msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void controlLoop(const ros::TimerEvent& event);

    // ROS communication
    ros::Subscriber pos_cmd_sub_;
    ros::Subscriber odom_sub_;
    ros::ServiceClient set_model_state_client_;
    ros::Timer control_timer_;

    // Parameters
    std::string model_name_;
    double control_frequency_;
    double command_timeout_;

    // State variables
    quadrotor_msgs::PositionCommand latest_cmd_;
    nav_msgs::Odometry current_odom_;
    bool received_cmd_;
    bool received_odom_;
    ros::Time last_cmd_time_;
};

} // namespace fast_planner_bridge

#endif // GAZEBO_CONTROL_BRIDGE_H
