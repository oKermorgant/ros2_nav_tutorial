#ifndef OCCUPANCYGRID_H
#define OCCUPANCYGRID_H

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <ros2_nav_tutorial/laser_scanner.h>

namespace ros2_nav_tutorial
{

class OccupancyGrid
{
  cv::Mat base_map, occ_map, scan_img;
  float resolution;
  float x0, y0;

  sensor_msgs::msg::LaserScan scan_msg;

  template <typename Numeric>
  cv::Point2f pointFrom(Numeric x, Numeric y)
  {
    return {(static_cast<float>(x)-x0)/resolution,
          occ_map.rows-(static_cast<float>(y)-y0)/resolution};
  }

  void writeScan(const std::vector<LaserScanner> &scanners, size_t idx);

public:
  OccupancyGrid()
  {
    cv::namedWindow("Simulator 2D", cv::WINDOW_NORMAL);
  }
  void initMap(const std::string &map_file, float max_height, float max_width);

  void simLaserScans(std::vector<LaserScanner> &scanners, const rclcpp::Time &now);
};

}

#endif // OCCUPANCYGRID_H
