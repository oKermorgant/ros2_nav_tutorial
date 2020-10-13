#include <ros2_nav_tutorial/occupancy_grid.h>
#include <yaml-cpp/yaml.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <experimental/filesystem>
#include <iostream>

namespace fs = std::experimental::filesystem;

namespace ros2_nav_tutorial
{

void OccupancyGrid::initMap(const std::string & map_file, float max_height, float max_width)
{
  std::cout << "Loading map" << std::endl;
  YAML::Node map_data = YAML::LoadFile(map_file);

  // map registration
  resolution = map_data["resolution"].as<float>();
  const auto origin(map_data["origin"].as<std::vector<float>>());
  x0 = origin[0];
  y0 = origin[1];

  // load and clean image
  std::string image(map_data["image"].as<std::string>());
  if(!fs::exists(image))
  {
    // relative to map file
    image = fs::path(map_file).remove_filename().string() + "/" + image;
  }
  std::cout << "Image: " << image << std::endl;

  occ_map = cv::imread(image, cv::IMREAD_GRAYSCALE);

  const double free_thr(map_data["free_thresh"].as<double>());
  const double occ_thr(map_data["occupied_thresh"].as<double>());
  const bool negate(map_data["negate"].as<int>());

  for(int i=0; i<occ_map.rows; i++)
  {
    for(int j=0; j<occ_map.cols; j++)
    {
      auto & pix(occ_map.at<uchar>(i,j));
      const double p = negate ? pix/255. : (255.-pix)/255.;

      if(p > occ_thr)
        pix = 0;
      else if(p < free_thr)
        pix = 255;
      else
      {
        pix = 255 - uchar(255*p);
      }
    }
  }

  // deal with max dimensions
  float scale = std::min(max_height/occ_map.rows, max_width/occ_map.cols);
  if(scale != 0.f && scale < 1)
  {
    cv::resizeWindow("Simulator 2D", int(scale*occ_map.rows), int(scale*occ_map.cols));
    std::cout << "Resizing display from " <<occ_map.cols << " x " << occ_map.rows
              << " to " << int(scale*occ_map.cols) << " x " << int(scale*occ_map.rows) << std::endl;

  }

  cv::cvtColor(occ_map, base_map, cv::COLOR_GRAY2BGR);

  scan_msg.ranges.resize(100);
  scan_msg.angle_min = -2*M_PI/3;
  scan_msg.angle_max = -scan_msg.angle_min;
  scan_msg.angle_increment = (scan_msg.angle_max - scan_msg.angle_min)/(scan_msg.ranges.size());
  scan_msg.range_min = 0.1;
  scan_msg.range_max = 5;
}

void OccupancyGrid::simLaserScans(std::vector<LaserScanner> &scanners, const rclcpp::Time &now)
{
  scan_img = base_map.clone();
  scan_msg.header.stamp = now;
  for(size_t idx = 0; idx < scanners.size(); idx++)
  {
    if(scanners[idx].active)
    {
      writeScan(scanners, idx);
      scanners[idx].publish(scan_msg);
    }
  }

  // display robots
  for(const auto &scanner: scanners)
  {
    if(scanner.active)
      cv::circle(scan_img, pointFrom(scanner.x, scanner.y), scanner.radius/resolution, scanner.color, -1);
  }

  cv::imshow("Simulator 2D", scan_img);
  cv::waitKey(1);
}

void OccupancyGrid::writeScan(const std::vector<LaserScanner> &scanners, size_t idx)
{
  const auto origin = pointFrom(scanners[idx].x, scanners[idx].y);
  float angle(scanners[idx].theta + scan_msg.angle_min);
  const auto max_pix_range(int(scan_msg.range_max / resolution));

  std::vector<std::pair<cv::Point2f, int>> other_robots;

  for(size_t other = 0; other < scanners.size(); ++other)
  {
    if(other != idx && scanners[other].active)
      other_robots.push_back({pointFrom(scanners[other].x, scanners[other].y),
                              std::pow(scanners[other].radius/resolution, 2)});
  }

  for(auto &range: scan_msg.ranges)
  {
    range = 0.;
    const float c(cos(angle));
    const float s(sin(angle));

    // ray tracing within image
    for(int k = 0; k < max_pix_range; ++k)
    {
      const auto u = int(origin.x + k*c);
      const auto v = int(origin.y - k*s);

      if(u < 0 || v < 0 || u >= occ_map.cols || v >= occ_map.rows)
        break;

      bool hit = occ_map.at<uchar>(v, u) == 0;

       if(!hit)
      {
        // test all other scanners
        for(const auto &[p, rad_sq]: other_robots)
        {
          if(std::pow(p.x-u,2) + std::pow(p.y-v,2) < rad_sq)
          {
            hit = true;
            break;
          }
        }
      }

      if(hit)
      {
        range = k*resolution;
        cv::line(scan_img, origin, {u, v}, scanners[idx].laser);
        break;
      }
    }
    angle += scan_msg.angle_increment;
  }
}
}
