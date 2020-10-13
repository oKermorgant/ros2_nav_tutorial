#include <ros2_nav_tutorial/simulation.h>
#include <iostream>

using namespace std::chrono;

namespace ros2_nav_tutorial
{

Simulation::Simulation(rclcpp::NodeOptions options)
  : rclcpp::Node("simulation", options)
{  
  //const auto map = declare_parameter<std::string>("map", "/home/olivier/code/ros2/src/ros2_nav_tutorial/maps/batS.yaml");
  //const auto max_height = declare_parameter<double>("max_height", 800);
  //const auto max_width = declare_parameter<double>("max_width", 1200);
  occupancy_grid.initMap("/home/olivier/code/ros2/src/ros2_nav_tutorial/maps/batS.yaml",
                         800, 1200);

  // considered scanners
  const std::vector<std::string> names{"bb8", "bb9", "d0", "d9"};
  scanners.reserve(names.size());
  for(const auto &name: names)
    scanners.emplace_back(this, name);

  // how often we scan for new / removed robots
  refresh_timer = create_wall_timer(
        1s, [&](){updateAvailableScanners();});

  // how often we update the scene
  const auto dt(0.2);
  publish_timer = create_wall_timer(milliseconds((long)(1000*dt)), [&]()
  {
    occupancy_grid.simLaserScans(scanners, now());});
}

void Simulation::updateAvailableScanners()
{
  // extract names of scanners being here
  std::vector<std::string> names;
  for(const auto &name: this->get_node_names())
  {
    auto ns = name.find("robot_state_publisher");
    if(ns != name.npos)
      names.push_back(name.substr(1, ns-2));
  }

  for(auto &scanner: scanners)
    scanner.activateFrom(names);
}
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(ros2_nav_tutorial::Simulation)
