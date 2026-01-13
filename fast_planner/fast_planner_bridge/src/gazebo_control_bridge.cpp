#include "fast_planner_bridge/gazebo_control_bridge.h"

namespace fast_planner_bridge {

GazeboControlBridge::GazeboControlBridge(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
    : received_cmd_(false), received_odom_(false) {

    // Load parameters
    nh_private.param<std::string>("model_name", model_name_, "quadrotor");
    nh_private.param<double>("control_frequency", control_frequency_, 100.0);
    nh_private.param<double>("command_timeout", command_timeout_, 0.5);

    ROS_INFO("[GazeboControlBridge] Initializing with model name: %s", model_name_.c_str());
    ROS_INFO("[GazeboControlBridge] Control frequency: %.1f Hz", control_frequency_);

    // Setup subscribers
    pos_cmd_sub_ = nh.subscribe("position_cmd", 10,
                                &GazeboControlBridge::positionCommandCallback, this);
    odom_sub_ = nh.subscribe("odom", 10,
                             &GazeboControlBridge::odomCallback, this);

    // Setup service client
    set_model_state_client_ = nh.serviceClient<gazebo_msgs::SetModelState>(
        "/gazebo/set_model_state");

    // Wait for Gazebo service
    ROS_INFO("[GazeboControlBridge] Waiting for /gazebo/set_model_state service...");
    set_model_state_client_.waitForExistence();
    ROS_INFO("[GazeboControlBridge] Service available!");

    // Setup control timer
    double timer_period = 1.0 / control_frequency_;
    control_timer_ = nh.createTimer(ros::Duration(timer_period),
                                    &GazeboControlBridge::controlLoop, this);

    ROS_INFO("[GazeboControlBridge] Initialized successfully!");
}

void GazeboControlBridge::positionCommandCallback(
    const quadrotor_msgs::PositionCommand::ConstPtr& msg) {
    latest_cmd_ = *msg;
    last_cmd_time_ = ros::Time::now();

    if (!received_cmd_) {
        ROS_INFO("[GazeboControlBridge] First position command received");
        received_cmd_ = true;
    }
}

void GazeboControlBridge::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    current_odom_ = *msg;

    if (!received_odom_) {
        ROS_INFO("[GazeboControlBridge] First odometry received");
        received_odom_ = true;
    }
}

void GazeboControlBridge::controlLoop(const ros::TimerEvent& event) {
    // Wait for both command and odometry
    if (!received_cmd_ || !received_odom_) {
        ROS_WARN_THROTTLE(5.0, "[GazeboControlBridge] Waiting for commands and odometry...");
        return;
    }

    // Check command timeout
    double time_since_cmd = (ros::Time::now() - last_cmd_time_).toSec();
    if (time_since_cmd > command_timeout_) {
        ROS_WARN_THROTTLE(1.0, "[GazeboControlBridge] No position command received for %.2f s",
                          time_since_cmd);
        return;
    }

    // Create ModelState message
    gazebo_msgs::ModelState model_state;
    model_state.model_name = model_name_;
    model_state.reference_frame = "world";

    // Set target position from Fast-Planner command
    model_state.pose.position.x = latest_cmd_.position.x;
    model_state.pose.position.y = latest_cmd_.position.y;
    model_state.pose.position.z = latest_cmd_.position.z;

    // Convert yaw to quaternion (assuming flat orientation)
    double yaw = latest_cmd_.yaw;
    model_state.pose.orientation.x = 0.0;
    model_state.pose.orientation.y = 0.0;
    model_state.pose.orientation.z = std::sin(yaw / 2.0);
    model_state.pose.orientation.w = std::cos(yaw / 2.0);

    // Set velocity for smooth movement
    model_state.twist.linear.x = latest_cmd_.velocity.x;
    model_state.twist.linear.y = latest_cmd_.velocity.y;
    model_state.twist.linear.z = latest_cmd_.velocity.z;
    model_state.twist.angular.x = 0.0;
    model_state.twist.angular.y = 0.0;
    model_state.twist.angular.z = latest_cmd_.yaw_dot;

    // Call Gazebo service
    gazebo_msgs::SetModelState srv;
    srv.request.model_state = model_state;

    if (!set_model_state_client_.call(srv)) {
        ROS_ERROR_THROTTLE(1.0, "[GazeboControlBridge] Failed to call SetModelState service");
        return;
    }

    if (!srv.response.success) {
        ROS_WARN_THROTTLE(1.0, "[GazeboControlBridge] SetModelState returned failure: %s",
                          srv.response.status_message.c_str());
    }
}

} // namespace fast_planner_bridge
