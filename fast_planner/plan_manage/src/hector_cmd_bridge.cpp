/**
 * Hector Command Bridge
 *
 * 將 Fast-Planner 的 quadrotor_msgs::PositionCommand 轉換成
 * Hector Quadrotor 的速度控制命令，確保精確跟蹤軌跡
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <hector_uav_msgs/EnableMotors.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>

class HectorCmdBridge {
public:
  HectorCmdBridge(ros::NodeHandle& nh, ros::NodeHandle& nh_global) {
    // Publishers
    // 使用 /cmd_vel 進行速度控制（更精確的軌跡跟蹤）
    vel_cmd_pub_ = nh_global.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    // 備用：位置控制（用於懸停）
    pose_cmd_pub_ = nh_global.advertise<geometry_msgs::PoseStamped>("/command/pose", 10);

    // Subscribers
    pos_cmd_sub_ = nh_global.subscribe("/position_cmd", 10,
                                 &HectorCmdBridge::positionCommandCallback, this);
    odom_sub_ = nh_global.subscribe("/ground_truth/state", 10,
                              &HectorCmdBridge::odomCallback, this);

    // Parameters
    nh.param<double>("max_linear_vel", max_linear_vel_, 2.0);
    nh.param<double>("max_angular_vel", max_angular_vel_, 1.5);
    nh.param<double>("position_gain", kp_, 1.5);  // 位置誤差增益
    nh.param<double>("velocity_gain", kv_, 0.5);  // 速度前饋增益

    // Enable motors service
    enable_motors_client_ = nh_global.serviceClient<hector_uav_msgs::EnableMotors>("/enable_motors");

    has_odom_ = false;
    motors_enabled_ = false;
    last_cmd_time_ = ros::Time(0);

    ROS_INFO("[HectorCmdBridge] Initialized with velocity control mode");
    ROS_INFO("[HectorCmdBridge] Kp=%.2f, Kv=%.2f, max_vel=%.2f", kp_, kv_, max_linear_vel_);
  }

  void enableMotors() {
    if (!motors_enabled_) {
      hector_uav_msgs::EnableMotors srv;
      srv.request.enable = true;
      if (enable_motors_client_.call(srv)) {
        if (srv.response.success) {
          motors_enabled_ = true;
          ROS_INFO("[HectorCmdBridge] Motors enabled!");
        }
      }
    }
  }

  void checkTimeout() {
    // 如果超過 0.5 秒沒有命令，發送零速度（懸停）
    if (has_odom_ && (ros::Time::now() - last_cmd_time_).toSec() > 0.5) {
      geometry_msgs::Twist zero_cmd;
      vel_cmd_pub_.publish(zero_cmd);
    }
  }

private:
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    current_odom_ = *msg;
    has_odom_ = true;

    // 提取當前 yaw
    tf2::Quaternion q(
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w
    );
    tf2::Matrix3x3 m(q);
    double roll, pitch;
    m.getRPY(roll, pitch, current_yaw_);
  }

  // 將世界座標系速度轉換到機體座標系
  void worldToBody(double vx_world, double vy_world, double yaw,
                   double& vx_body, double& vy_body) {
    double cos_yaw = std::cos(yaw);
    double sin_yaw = std::sin(yaw);
    // 旋轉矩陣的逆（世界到機體）
    vx_body =  cos_yaw * vx_world + sin_yaw * vy_world;
    vy_body = -sin_yaw * vx_world + cos_yaw * vy_world;
  }

  void positionCommandCallback(const quadrotor_msgs::PositionCommand::ConstPtr& cmd) {
    if (!has_odom_) {
      ROS_WARN_THROTTLE(1.0, "[HectorCmdBridge] No odometry yet");
      return;
    }

    enableMotors();
    last_cmd_time_ = ros::Time::now();

    // 計算位置誤差（世界座標系）
    double ex = cmd->position.x - current_odom_.pose.pose.position.x;
    double ey = cmd->position.y - current_odom_.pose.pose.position.y;
    double ez = cmd->position.z - current_odom_.pose.pose.position.z;

    // 計算期望速度（世界座標系）= 位置誤差 * Kp + 前饋速度 * Kv
    double vx_world = kp_ * ex + kv_ * cmd->velocity.x;
    double vy_world = kp_ * ey + kv_ * cmd->velocity.y;
    double vz_world = kp_ * ez + kv_ * cmd->velocity.z;

    // 速度限幅（世界座標系）
    double v_mag = std::sqrt(vx_world*vx_world + vy_world*vy_world + vz_world*vz_world);
    if (v_mag > max_linear_vel_) {
      double scale = max_linear_vel_ / v_mag;
      vx_world *= scale;
      vy_world *= scale;
      vz_world *= scale;
    }

    // 將 XY 速度從世界座標系轉換到機體座標系
    double vx_body, vy_body;
    worldToBody(vx_world, vy_world, current_yaw_, vx_body, vy_body);

    // 計算 yaw 誤差並轉換為角速度
    double yaw_error = cmd->yaw - current_yaw_;
    // 歸一化到 [-pi, pi]
    while (yaw_error > M_PI) yaw_error -= 2.0 * M_PI;
    while (yaw_error < -M_PI) yaw_error += 2.0 * M_PI;
    double yaw_rate = 2.0 * yaw_error;  // P 控制
    if (yaw_rate > max_angular_vel_) yaw_rate = max_angular_vel_;
    if (yaw_rate < -max_angular_vel_) yaw_rate = -max_angular_vel_;

    // 發布速度命令（機體座標系 - Hector /cmd_vel 使用機體座標系）
    geometry_msgs::Twist vel_cmd;
    vel_cmd.linear.x = vx_body;   // 前進/後退
    vel_cmd.linear.y = vy_body;   // 左/右
    vel_cmd.linear.z = vz_world;  // 上/下（世界座標系）
    vel_cmd.angular.z = yaw_rate;
    vel_cmd_pub_.publish(vel_cmd);

    // Debug 輸出
    ROS_DEBUG_THROTTLE(0.5, "[HectorCmdBridge] world=(%.2f,%.2f) body=(%.2f,%.2f) yaw=%.2f",
              vx_world, vy_world, vx_body, vy_body, current_yaw_);
  }

  ros::Publisher vel_cmd_pub_;
  ros::Publisher pose_cmd_pub_;
  ros::Subscriber pos_cmd_sub_;
  ros::Subscriber odom_sub_;
  ros::ServiceClient enable_motors_client_;

  nav_msgs::Odometry current_odom_;
  double current_yaw_;
  bool has_odom_;
  bool motors_enabled_;
  ros::Time last_cmd_time_;

  double max_linear_vel_;
  double max_angular_vel_;
  double kp_;  // 位置增益
  double kv_;  // 速度前饋增益
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "hector_cmd_bridge");
  ros::NodeHandle nh("~");
  ros::NodeHandle nh_global;

  HectorCmdBridge bridge(nh, nh_global);

  ros::Rate rate(50);  // 50 Hz 控制迴圈
  while (ros::ok()) {
    bridge.checkTimeout();
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
