#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <curl/curl.h>

#include "lidar/data_type.h"

#define DEG2RAD(x) ((x) * M_PI / 180.0f)

namespace
{

constexpr const char * kHostIp = "0.0.0.0";
constexpr const char * kLeftSensorIp = "192.168.30.10";
constexpr const char * kRightSensorIp = "192.168.30.11";
constexpr const char * kLeftFrame = "lidar_left";
constexpr const char * kRightFrame = "lidar_right";
constexpr const char * kOutputFrame = "base_link";
constexpr const char * kOutputTopic = "scan";
constexpr int kLeftPort = 2367;
constexpr int kRightPort = 2368;
constexpr int64_t kScanFreq = 30;
constexpr bool kLaserEnable = true;
constexpr int64_t kScanRangeStart = 45;
constexpr int64_t kScanRangeStop = 315;

// Fixed LiDAR poses in base_link, copied from urdf/rby1_base.urdf.
constexpr double kLeftX = 0.228;
constexpr double kLeftY = 0.1765;
constexpr double kLeftYaw = 0.9599;
constexpr double kRightX = 0.228;
constexpr double kRightY = -0.1765;
constexpr double kRightYaw = -0.9599;

size_t dummy_callback(void *, size_t size, size_t nmemb, void *)
{
  return size * nmemb;
}

int sensor_config(
  const std::string & sensor_ipaddr,
  const std::string & parameter,
  const std::string & value,
  const rclcpp::Logger & logger)
{
  std::string url = "http://" + sensor_ipaddr + parameter;
  RCLCPP_INFO(logger, "sensor_config: PUT %s = %s", url.c_str(), value.c_str());

  CURL * curl = curl_easy_init();
  if (!curl) {
    RCLCPP_WARN(logger, "sensor_config: curl_easy_init failed");
    return -1;
  }

  curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
  curl_easy_setopt(curl, CURLOPT_TIMEOUT, 3L);
  curl_easy_setopt(curl, CURLOPT_CUSTOMREQUEST, "PUT");
  curl_easy_setopt(curl, CURLOPT_POSTFIELDS, value.c_str());
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, dummy_callback);

  CURLcode res = curl_easy_perform(curl);
  int ret = 0;
  if (res == CURLE_OK) {
    long http_code = 0;
    curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_code);
    if (http_code == 200) {
      RCLCPP_INFO(logger, "sensor_config: %s = %s ... OK", url.c_str(), value.c_str());
    } else {
      RCLCPP_WARN(logger, "sensor_config: %s = %s ... HTTP %ld", url.c_str(), value.c_str(), http_code);
      ret = -1;
    }
  } else {
    RCLCPP_WARN(logger, "sensor_config: curl error [%s] - check LiDAR connection", curl_easy_strerror(res));
    ret = -1;
  }

  curl_easy_cleanup(curl);
  return ret;
}

struct FixedPose
{
  double x;
  double y;
  double yaw;
};

struct LidarConfig
{
  std::string name;
  std::string sensor_ip;
  int port;
  std::string frame_id;
  FixedPose pose;
};

class LidarReceiver
{
public:
  LidarReceiver(
    rclcpp::Node * node,
    const LidarConfig & config)
  : node_(node),
    logger_(node->get_logger()),
    config_(config)
  {
  }

  ~LidarReceiver()
  {
    stop();
  }

  bool start()
  {
    if (!create_socket()) {
      return false;
    }

    thread_ = std::thread(&LidarReceiver::scan_loop, this);
    return true;
  }

  void stop()
  {
    if (sockfd_ >= 0) {
      close(sockfd_);
      sockfd_ = -1;
    }
    if (thread_.joinable()) {
      thread_.join();
    }
  }

  bool latest(sensor_msgs::msg::LaserScan & scan)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!latest_scan_) {
      return false;
    }
    scan = *latest_scan_;
    return true;
  }

  const FixedPose & pose() const
  {
    return config_.pose;
  }

private:
  bool create_socket()
  {
    sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd_ < 0) {
      RCLCPP_ERROR(logger_, "[%s] socket() failed: %s", config_.name.c_str(), strerror(errno));
      return false;
    }

    struct timeval tv;
    tv.tv_sec = 1;
    tv.tv_usec = 0;
    setsockopt(sockfd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = inet_addr(kHostIp);
    addr.sin_port = htons(static_cast<uint16_t>(config_.port));

    if (bind(sockfd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
      RCLCPP_ERROR(
        logger_, "[%s] bind() failed on %s:%d - %s",
        config_.name.c_str(), kHostIp, config_.port, strerror(errno));
      close(sockfd_);
      sockfd_ = -1;
      return false;
    }

    RCLCPP_INFO(logger_, "[%s] UDP socket bound to %s:%d", config_.name.c_str(), kHostIp, config_.port);
    return true;
  }

  void scan_loop()
  {
    const double inf = std::numeric_limits<double>::infinity();
    MSOP_Packet pkt;
    sockaddr_in client_addr;
    socklen_t addr_len = sizeof(client_addr);
    std::vector<bm_response_scan_t> scan_vec;
    rclcpp::Time scan_begin;
    rclcpp::Time scan_end;
    int i = 0;
    int j = 12;
    int resolution = 25;
    bool scan_ready = false;

    RCLCPP_INFO(logger_, "[%s] scan loop started", config_.name.c_str());

    while (rclcpp::ok()) {
      if (!scan_ready) {
        while (rclcpp::ok()) {
          if (j == 12) {
            ssize_t n = recvfrom(
              sockfd_, &pkt, sizeof(pkt), 0,
              reinterpret_cast<sockaddr *>(&client_addr), &addr_len);
            if (n < 0) {
              if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EBADF) {
                break;
              }
              RCLCPP_WARN(logger_, "[%s] recvfrom error: %s", config_.name.c_str(), strerror(errno));
              break;
            }

            if (pkt.BlockID[0].Azimuth == 0) {
              scan_end = scan_begin;
              scan_begin = node_->now();
            }

            if (pkt.BlockID[1].Azimuth > pkt.BlockID[0].Azimuth) {
              resolution = (pkt.BlockID[1].Azimuth - pkt.BlockID[0].Azimuth) / 16;
            }
            j = 0;
          }

          for (; j < 12; j++) {
            for (i = 0; i < 16; i++) {
              if (pkt.BlockID[j].DataFlag != 0xEEFF) {
                continue;
              }

              bm_response_scan_t resp;
              resp.angle = pkt.BlockID[j].Azimuth + (resolution * i);
              if (resp.angle == 0 && !scan_vec.empty()) {
                scan_ready = true;
                if (scan_vec.size() < 1200) {
                  j = 12;
                }
                break;
              }

              resp.dist = pkt.BlockID[j].Result[i].Dist_1;
              resp.rssi = pkt.BlockID[j].Result[i].RSSI_1;
              scan_vec.push_back(resp);
            }
            if (scan_ready) {
              break;
            }
          }
          if (scan_ready) {
            break;
          }
        }
      }

      if (scan_ready) {
        const uint32_t num_readings = static_cast<uint32_t>(scan_vec.size());
        if (num_readings == 0) {
          scan_ready = false;
          continue;
        }

        float duration = static_cast<float>((scan_begin - scan_end).seconds());

        sensor_msgs::msg::LaserScan scan;
        scan.header.stamp = scan_begin;
        scan.header.frame_id = config_.frame_id;
        scan.angle_min = DEG2RAD(-180);
        scan.angle_max = DEG2RAD(180);
        scan.angle_increment = 2.0f * static_cast<float>(M_PI) / num_readings;
        scan.scan_time = duration;
        scan.time_increment = duration / static_cast<float>(num_readings) / 2.0f;
        scan.range_min = 0.0f;
        scan.range_max = 100.0f;
        scan.ranges.resize(num_readings);
        scan.intensities.resize(num_readings);

        for (uint32_t idx = 0; idx < num_readings; ++idx) {
          float range = static_cast<float>(scan_vec[idx].dist) / 1000.0f;
          if (range == 0.0f) {
            scan.ranges[idx] = static_cast<float>(inf);
            scan.intensities[idx] = 0.0f;
          } else {
            scan.ranges[idx] = range;
            scan.intensities[idx] = static_cast<float>(scan_vec[idx].rssi);
          }
        }

        {
          std::lock_guard<std::mutex> lock(mutex_);
          latest_scan_ = std::make_shared<sensor_msgs::msg::LaserScan>(scan);
        }

        scan_vec.clear();
        scan_ready = false;
      }
    }

    RCLCPP_INFO(logger_, "[%s] scan loop exited", config_.name.c_str());
  }

  rclcpp::Node * node_;
  rclcpp::Logger logger_;
  LidarConfig config_;
  sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
  std::mutex mutex_;
  std::thread thread_;
  int sockfd_ = -1;
};

class DualLidarNode : public rclcpp::Node
{
public:
  DualLidarNode()
  : Node("dual_lidar_node")
  {
    declare_parameter<bool>("configure_sensor", true);

    bool configure_sensor = get_parameter("configure_sensor").as_bool();

    left_config_ = {
      "left",
      kLeftSensorIp,
      kLeftPort,
      kLeftFrame,
      {kLeftX, kLeftY, kLeftYaw},
    };
    right_config_ = {
      "right",
      kRightSensorIp,
      kRightPort,
      kRightFrame,
      {kRightX, kRightY, kRightYaw},
    };

    RCLCPP_INFO(
      get_logger(), "dual LiDAR -> /%s (frame=%s)",
      kOutputTopic, kOutputFrame);

    if (configure_sensor) {
      configure_lidar(left_config_);
      configure_lidar(right_config_);
    }

    pub_ = create_publisher<sensor_msgs::msg::LaserScan>(kOutputTopic, rclcpp::QoS(5).reliable());

    left_ = std::make_unique<LidarReceiver>(this, left_config_);
    right_ = std::make_unique<LidarReceiver>(this, right_config_);

    bool left_ok = left_->start();
    bool right_ok = right_->start();
    if (!left_ok || !right_ok) {
      RCLCPP_ERROR(get_logger(), "dual LiDAR startup incomplete: left=%s right=%s", left_ok ? "ok" : "fail", right_ok ? "ok" : "fail");
    }

    timer_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&DualLidarNode::try_merge, this));
  }

private:
  void configure_lidar(const LidarConfig & config)
  {
    RCLCPP_INFO(get_logger(), "[%s] configuring LiDAR via HTTP ...", config.name.c_str());
    sensor_config(config.sensor_ip, "/api/v1/sensor/scanfreq", std::to_string(kScanFreq), get_logger());
    sensor_config(config.sensor_ip, "/api/v1/sensor/laser_enable", kLaserEnable ? "true" : "false", get_logger());
    sensor_config(config.sensor_ip, "/api/v1/sensor/scan_range/start", std::to_string(kScanRangeStart), get_logger());
    sensor_config(config.sensor_ip, "/api/v1/sensor/scan_range/stop", std::to_string(kScanRangeStop), get_logger());
  }

  static void fill_scan(
    const sensor_msgs::msg::LaserScan & scan,
    const FixedPose & pose,
    sensor_msgs::msg::LaserScan & out)
  {
    double cy = std::cos(pose.yaw);
    double sy = std::sin(pose.yaw);
    double angle = scan.angle_min;

    for (float r : scan.ranges) {
      if (std::isfinite(r) && r > 0.0f) {
        double x = r * std::cos(angle);
        double y = r * std::sin(angle);
        double bx = pose.x + x * cy - y * sy;
        double by = pose.y + x * sy + y * cy;
        double br = std::hypot(bx, by);

        if (br >= out.range_min && br <= out.range_max) {
          int idx = static_cast<int>(std::round((std::atan2(by, bx) - out.angle_min) / out.angle_increment));
          if (idx >= 0 && idx < static_cast<int>(out.ranges.size()) && static_cast<float>(br) < out.ranges[idx]) {
            out.ranges[idx] = static_cast<float>(br);
          }
        }
      }
      angle += scan.angle_increment;
    }
  }

  void try_merge()
  {
    sensor_msgs::msg::LaserScan left_scan;
    sensor_msgs::msg::LaserScan right_scan;
    if (!left_->latest(left_scan) || !right_->latest(right_scan)) {
      return;
    }

    sensor_msgs::msg::LaserScan out;
    out.header.frame_id = kOutputFrame;
    out.header.stamp = rclcpp::Time(left_scan.header.stamp) >= rclcpp::Time(right_scan.header.stamp)
      ? left_scan.header.stamp
      : right_scan.header.stamp;

    double inc0 = std::abs(left_scan.angle_increment);
    double inc1 = std::abs(right_scan.angle_increment);
    out.angle_min = -M_PI;
    out.angle_max = M_PI;
    out.angle_increment = (inc0 > 0.0 && inc1 > 0.0) ? std::min(inc0, inc1) :
      (inc0 > 0.0 ? inc0 : (inc1 > 0.0 ? inc1 : M_PI / 360.0));
    out.range_min = std::min(left_scan.range_min, right_scan.range_min);
    out.range_max = std::max(left_scan.range_max, right_scan.range_max);
    out.time_increment = 0.0f;
    out.scan_time = 0.0f;

    int n = static_cast<int>(std::floor((out.angle_max - out.angle_min) / out.angle_increment)) + 1;
    out.ranges.assign(n, std::numeric_limits<float>::infinity());

    fill_scan(left_scan, left_->pose(), out);
    fill_scan(right_scan, right_->pose(), out);

    pub_->publish(out);
  }

  LidarConfig left_config_;
  LidarConfig right_config_;
  std::unique_ptr<LidarReceiver> left_;
  std::unique_ptr<LidarReceiver> right_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DualLidarNode>());
  rclcpp::shutdown();
  return 0;
}
