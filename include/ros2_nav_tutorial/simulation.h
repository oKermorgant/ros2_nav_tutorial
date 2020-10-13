#ifndef TUTO_SIMULATION_H
#define TUTO_SIMULATION_H

#include "rclcpp/rclcpp.hpp"
#include <ros2_nav_tutorial/laser_scanner.h>
#include <ros2_nav_tutorial/occupancy_grid.h>
#include <chrono>

namespace ros2_nav_tutorial
{
    
class Simulation : public rclcpp::Node
{
public:
    Simulation(rclcpp::NodeOptions options);

private:
  void updateAvailableScanners();
  std::vector<LaserScanner> scanners;

  OccupancyGrid occupancy_grid;

  rclcpp::TimerBase::SharedPtr refresh_timer, publish_timer;
};

}

#endif // TUTO_SIMULATION_H
