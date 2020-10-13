#include <ros2_nav_tutorial/robot.h>
#include <chrono>

using namespace std::chrono;

namespace ros2_nav_tutorial
{

Robot::Robot(rclcpp::NodeOptions options)
  : rclcpp::Node("robot", options), br(*this, 10)
{
  joints.name = {"wheel", "torso", "neck"};
  joints.position = {0,0,0};

  std::string name(get_namespace());
  name = name.substr(1, name.npos);
  tf.header.frame_id = name + "/odom";
  tf.child_frame_id = name + "/base_link";

  if(name == "bb9")
  {
    pose.x = 14.664304733276367;
    pose.y = 15.193315505981445;
    pose.theta = 2.736574156107229;
  }
  else if(name == "d0")
  {
    pose.x = 13.159404754638672;
    pose.y = 14.023484230041504;
    pose.theta = 2.736574156107229;
  }
  else if(name == "d9")
  {
    pose.x = 2.3937320709228516;
    pose.y = -7.750550746917725;
    pose.theta = 0.88381512047772;
  }

  cmd_subscriber = create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel",
        10,
        [this](geometry_msgs::msg::Twist::UniquePtr msg)
  {
      v = msg->linear.x;
      w = msg->angular.z;
});

  gt_publisher = create_publisher<geometry_msgs::msg::Pose2D>("ground_truth", 10);
  joint_publisher = create_publisher<sensor_msgs::msg::JointState>
      ("joint_states", 10);

  publish_timer = create_wall_timer(milliseconds((long)(1000*dt)), [&]()
  {publishAll();});

  // service example
  change_mode_srv = create_service<ros2_nav_tutorial::srv::ChangeMode>
      ("change_mode", [&](const std::shared_ptr<ModeRequest> request, std::shared_ptr<srv::ChangeMode_Response> response)
  {
    (void)(response);
    current_mode = request->mode;
  });
}

void Robot::publishAll()
{
  updatePose();
  gt_publisher->publish(pose);

  // publish corresponding tf
  tf.transform.translation.x = pose.x;
  tf.transform.translation.y = pose.y;
  tf.transform.rotation.z = sin(pose.theta/2);
  tf.transform.rotation.w = cos(pose.theta/2);
  tf.header.stamp = now();
  br.sendTransform(tf);

  if(current_mode != ModeRequest::CMD_VEL_NO_JOINTS)
  {
    joints.position[0] += 3.7*v*dt;
    joints.position[1] = v*M_PI/12;
    joints.position[2] = w*M_PI/12;
    joints.header.stamp = now();
    joint_publisher->publish(joints);
  }
}


}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(ros2_nav_tutorial::Robot)
