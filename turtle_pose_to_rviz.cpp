#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"

static geometry_msgs::msg::Quaternion yawToQuat(double yaw) {
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0; q.y = 0.0;
  q.z = std::sin(yaw * 0.5);
  q.w = std::cos(yaw * 0.5);
  return q;
}

class TurtlePoseToRviz : public rclcpp::Node {
public:
  TurtlePoseToRviz()
  : Node("turtle_pose_to_rviz"),
    frame_map_(declare_parameter<std::string>("map_frame", "map")),
    frame_base_(declare_parameter<std::string>("base_frame", "turtle1/base_link")) {
    pose_sub_ = create_subscription<turtlesim::msg::Pose>(
      "/turtle1/pose", 10, std::bind(&TurtlePoseToRviz::poseCb, this, std::placeholders::_1));
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/turtle1/pose_stamped", 10);
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/turtle1/odom", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    RCLCPP_INFO(get_logger(), "turtle_pose_to_rviz started");
  }

private:
  void poseCb(const turtlesim::msg::Pose::SharedPtr msg) {
    rclcpp::Time now = this->now();

    geometry_msgs::msg::PoseStamped ps;
    ps.header.stamp = now;
    ps.header.frame_id = frame_map_;
    ps.pose.position.x = msg->x;
    ps.pose.position.y = msg->y;
    ps.pose.orientation = yawToQuat(msg->theta);
    pose_pub_->publish(ps);

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = now;
    odom.header.frame_id = frame_map_;
    odom.child_frame_id = frame_base_;
    odom.pose.pose = ps.pose;
    odom.twist.twist.linear.x = msg->linear_velocity;
    odom.twist.twist.angular.z = msg->angular_velocity;
    odom_pub_->publish(odom);

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = now;
    tf.header.frame_id = frame_map_;
    tf.child_frame_id = frame_base_;
    tf.transform.translation.x = msg->x;
    tf.transform.translation.y = msg->y;
    tf.transform.rotation = ps.pose.orientation;
    tf_broadcaster_->sendTransform(tf);
  }

  std::string frame_map_, frame_base_;
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtlePoseToRviz>());
  rclcpp::shutdown();
  return 0;
}