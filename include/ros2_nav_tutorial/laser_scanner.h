#ifndef TUTO_LASER_SCANNER
#define TUTO_LASER_SCANNER

#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <opencv2/core/types.hpp>

struct LaserScanner
{
  std::string name;
  double x, y, theta;
  double radius;
  bool active = false;
  cv::Scalar laser{0,0,255};
  cv::Scalar color{0,0,0};
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_publisher;
  rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr gt_subscriber;

  LaserScanner(rclcpp::Node *sim, std::string _name)
    : name(_name), radius(name[0] == 'b' ? .27 : .16)
  {
    scan_publisher = sim->create_publisher<sensor_msgs::msg::LaserScan>(name + "/scan", 10);

    gt_subscriber = sim->create_subscription<geometry_msgs::msg::Pose2D>(name + "/ground_truth", 10,
                                                                         [this](geometry_msgs::msg::Pose2D::UniquePtr msg)
    {
        x = msg->x;
        y = msg->y;
        theta = msg->theta;
  });

    if(name == "bb8" || name == "d0")
      color = cv::Scalar{170,170,170};

    if(name == "bb8")
      laser = cv::Scalar{0,0,255};
    else if(name == "d0")
      laser = cv::Scalar{30,255,0};
  }
  void publish(sensor_msgs::msg::LaserScan &scan)
  {
    scan.header.frame_id = name + "/scan";
    scan_publisher->publish(scan);
  }

  void activateFrom(const std::vector<std::string> &names)
  {
    active = std::find(names.begin(), names.end(), name) != names.end();
  }

};
#endif
