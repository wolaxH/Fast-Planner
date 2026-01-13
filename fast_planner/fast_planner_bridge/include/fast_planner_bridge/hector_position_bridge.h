#ifndef HECTOR_POSITION_BRIDGE_H
#define HECTOR_POSITION_BRIDGE_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace fast_planner_bridge {

class HectorPositionBridge {
public:
    HectorPositionBridge(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
    ~HectorPositionBridge() = default;

private:
    // Callback functions
    void positionCommandCallback(const quadrotor_msgs::PositionCommand::ConstPtr& msg);
    void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg);

    // ROS communication
    ros::Subscriber pos_cmd_sub_;
    ros::Subscriber odom_sub_;
    ros::Publisher pose_cmd_pub_;

    // State
    double current_yaw_;
    bool has_odom_;

    // Parameters
    double command_timeout_;
    ros::Time last_cmd_time_;
    bool received_cmd_;
};

} // namespace fast_planner_bridge

#endif // HECTOR_POSITION_BRIDGE_H
