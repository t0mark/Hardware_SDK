#ifndef __LIDAR_REMOTE_H__
#define __LIDAR_REMOTE_H__

#include <string>
#include <rclcpp/rclcpp.hpp>

// Configure a single sensor parameter via HTTP PUT.
// Returns 0 on success, -1 on curl/HTTP error.
int sensor_config(const std::string& sensor_ipaddr,
                  const std::string& parameter,
                  const std::string& value,
                  const rclcpp::Logger& logger);

#endif  // __LIDAR_REMOTE_H__
