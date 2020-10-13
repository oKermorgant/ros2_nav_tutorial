#ifndef ROBOT_H
#define ROBOT_H

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <ros2_nav_tutorial/srv/change_mode.hpp>

namespace ros2_nav_tutorial
{

class Robot : public rclcpp::Node
{
  using ModeRequest  = srv::ChangeMode_Request;

public:
  Robot(rclcpp::NodeOptions options);
private:
  void updatePose()
  {
    pose.x += v*cos(pose.theta)*dt;
    pose.y += v*sin(pose.theta)*dt;
    pose.theta += w*dt;
  }

  void publishAll();

  // basic topics with member function
  double v = 0, w = 0;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_subscriber;
  rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr gt_publisher;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_publisher;

  // service with lambda
  uint8_t current_mode= ModeRequest::CMD_VEL;
  rclcpp::Service<ros2_nav_tutorial::srv::ChangeMode>::SharedPtr change_mode_srv;


  rclcpp::TimerBase::SharedPtr publish_timer;

  geometry_msgs::msg::Pose2D pose;
  sensor_msgs::msg::JointState joints;

  tf2_ros::TransformBroadcaster br;
  geometry_msgs::msg::TransformStamped tf;

  const double dt = 0.02;
};

}

#endif // ROBOT_H
