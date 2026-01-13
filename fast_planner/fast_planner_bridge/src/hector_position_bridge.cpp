#include "fast_planner_bridge/hector_position_bridge.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>

namespace fast_planner_bridge {

HectorPositionBridge::HectorPositionBridge(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
    : received_cmd_(false), has_odom_(false), current_yaw_(0.0) {

    // Get parameters
    nh_private.param("command_timeout", command_timeout_, 0.5);

    // Subscribe to Fast-Planner position commands
    pos_cmd_sub_ = nh.subscribe("/planning/pos_cmd", 10,
                                &HectorPositionBridge::positionCommandCallback, this);

    // Subscribe to odometry for current yaw
    odom_sub_ = nh.subscribe("/ground_truth/state", 10,
                             &HectorPositionBridge::odometryCallback, this);

    // Publish to Hector position controller
    pose_cmd_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/command/pose", 10);

    ROS_INFO("[HectorPositionBridge] Initialized successfully!");
    ROS_INFO("[HectorPositionBridge] Subscribing to: /planning/pos_cmd, /ground_truth/state");
    ROS_INFO("[HectorPositionBridge] Publishing pose to: /command/pose with velocity-aligned yaw");
}

void HectorPositionBridge::odometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    // Extract yaw from quaternion
    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w
    );

    tf2::Matrix3x3 m(q);
    double roll, pitch;
    m.getRPY(roll, pitch, current_yaw_);

    has_odom_ = true;
}

void HectorPositionBridge::positionCommandCallback(
    const quadrotor_msgs::PositionCommand::ConstPtr& msg) {

    if (!received_cmd_) {
        ROS_INFO("[HectorPositionBridge] First position command received");
        received_cmd_ = true;
    }

    if (!has_odom_) {
        ROS_WARN_THROTTLE(1.0, "[HectorPositionBridge] Waiting for odometry...");
        return;
    }

    last_cmd_time_ = ros::Time::now();

    // Simple direct forwarding of Fast-Planner commands
    geometry_msgs::PoseStamped pose_cmd;
    pose_cmd.header.stamp = ros::Time::now();
    pose_cmd.header.frame_id = "world";

    // Use Fast-Planner's commanded position and yaw directly
    pose_cmd.pose.position.x = msg->position.x;
    pose_cmd.pose.position.y = msg->position.y;
    pose_cmd.pose.position.z = msg->position.z;

    // Use Fast-Planner's commanded yaw
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, msg->yaw);
    pose_cmd.pose.orientation = tf2::toMsg(q);

    pose_cmd_pub_.publish(pose_cmd);
}

} // namespace fast_planner_bridge
