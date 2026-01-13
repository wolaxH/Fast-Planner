#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

class OdomToTF {
public:
    OdomToTF() {
        ros::NodeHandle nh;
        ros::NodeHandle nh_private("~");

        // Get parameters
        std::string odom_topic = nh_private.param<std::string>("odom_topic", "/ground_truth/state");
        world_frame_ = nh_private.param<std::string>("world_frame", "world");
        base_frame_ = nh_private.param<std::string>("base_frame", "base_link");

        // Subscribe to odometry
        odom_sub_ = nh.subscribe(odom_topic, 10, &OdomToTF::odomCallback, this);

        ROS_INFO("[OdomToTF] Subscribed to %s", odom_topic.c_str());
        ROS_INFO("[OdomToTF] Broadcasting TF: %s -> %s", world_frame_.c_str(), base_frame_.c_str());
    }

private:
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        geometry_msgs::TransformStamped transform;

        transform.header.stamp = msg->header.stamp;
        transform.header.frame_id = world_frame_;
        transform.child_frame_id = base_frame_;

        // Copy position
        transform.transform.translation.x = msg->pose.pose.position.x;
        transform.transform.translation.y = msg->pose.pose.position.y;
        transform.transform.translation.z = msg->pose.pose.position.z;

        // Copy orientation
        transform.transform.rotation = msg->pose.pose.orientation;

        // Broadcast TF
        tf_broadcaster_.sendTransform(transform);
    }

    ros::Subscriber odom_sub_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    std::string world_frame_;
    std::string base_frame_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "odom_to_tf_node");

    ROS_INFO("Starting Odometry to TF Broadcaster Node...");

    OdomToTF odom_to_tf;

    ros::spin();

    return 0;
}
