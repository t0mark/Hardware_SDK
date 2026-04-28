#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <chrono>
#include <cstring>
#include <cstdint>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

namespace
{

const std::map<std::string, std::pair<int, int>> kResolutionMap = {
  {"HD2K", {4416, 1242}},
  {"HD1080", {3840, 1080}},
  {"HD720", {2560, 720}},
  {"VGA", {1344, 376}},
};

constexpr const char * kFrameId = "head_camera";
constexpr const char * kTopicName = "camera/image_raw";

}  // namespace

class CameraNode : public rclcpp::Node
{
public:
  CameraNode()
  : Node("camera_node")
  {
    device_ = static_cast<int>(declare_parameter<int64_t>("device", 0));
    fps_ = static_cast<int>(std::max<int64_t>(1, declare_parameter<int64_t>("fps", 30)));
    resolution_ = declare_parameter<std::string>("resolution", "HD720");

    open_camera();

    image_pub_ = create_publisher<sensor_msgs::msg::Image>(
      kTopicName, rclcpp::SensorDataQoS());

    const auto period = std::chrono::duration<double>(1.0 / static_cast<double>(fps_));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() { capture_and_publish(); });

    RCLCPP_INFO(
      get_logger(), "ZED2i camera publisher started: topic=%s, frame_id=%s",
      image_pub_->get_topic_name(), kFrameId);
  }

  ~CameraNode() override
  {
    if (cap_.isOpened()) {
      cap_.release();
    }
  }

private:
  void open_camera()
  {
    auto it = kResolutionMap.find(resolution_);
    if (it == kResolutionMap.end()) {
      RCLCPP_WARN(
        get_logger(), "Unknown resolution '%s'. Falling back to HD720.", resolution_.c_str());
      it = kResolutionMap.find("HD720");
    }

    const int width = it->second.first;
    const int height = it->second.second;

    cap_.open(device_, cv::CAP_V4L2);
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, width);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height);
    cap_.set(cv::CAP_PROP_FPS, fps_);

    if (!cap_.isOpened()) {
      throw std::runtime_error("Failed to open camera device " + std::to_string(device_));
    }

    const int actual_width = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_WIDTH));
    const int actual_height = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_HEIGHT));
    RCLCPP_INFO(
      get_logger(), "Camera opened: device=%d, requested=%dx%d, actual=%dx%d @ %dfps",
      device_, width, height, actual_width, actual_height, fps_);
  }

  void capture_and_publish()
  {
    cv::Mat frame;
    if (!cap_.read(frame) || frame.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Failed to read camera frame");
      return;
    }

    cv::Mat image = frame(cv::Rect(0, 0, frame.cols / 2, frame.rows));

    if (!image.isContinuous()) {
      image = image.clone();
    }

    sensor_msgs::msg::Image msg;
    msg.header.stamp = now();
    msg.header.frame_id = kFrameId;
    msg.height = static_cast<uint32_t>(image.rows);
    msg.width = static_cast<uint32_t>(image.cols);
    msg.encoding = "bgr8";
    msg.is_bigendian = false;
    msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(image.cols * image.elemSize());
    msg.data.resize(static_cast<size_t>(msg.step) * msg.height);
    std::memcpy(msg.data.data(), image.data, msg.data.size());
    image_pub_->publish(std::move(msg));
  }

  int device_;
  int fps_;
  std::string resolution_;

  cv::VideoCapture cap_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<CameraNode>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("camera_node"), "%s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
