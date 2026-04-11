#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <cmath>
#include <limits>
#include <vector>
#include <string>

class MergeLaserScanNode : public rclcpp::Node
{
public:
    MergeLaserScanNode()
    : Node("lidar_merge_node")
    {
        declare_parameter<std::string>("scan0_topic",    "scan_left");
        declare_parameter<std::string>("scan1_topic",    "scan_right");
        declare_parameter<std::string>("output_topic",   "scan");
        declare_parameter<std::string>("target_frame",   "base_link");
        declare_parameter<double>     ("tf_timeout_sec", 0.2);

        scan0_topic_    = get_parameter("scan0_topic").as_string();
        scan1_topic_    = get_parameter("scan1_topic").as_string();
        output_topic_   = get_parameter("output_topic").as_string();
        target_frame_   = get_parameter("target_frame").as_string();
        tf_timeout_sec_ = get_parameter("tf_timeout_sec").as_double();

        rclcpp::QoS qos(5);
        qos.reliable();

        sub0_ = create_subscription<sensor_msgs::msg::LaserScan>(
            scan0_topic_, qos,
            [this](sensor_msgs::msg::LaserScan::SharedPtr msg) { s0_ = msg; });
        sub1_ = create_subscription<sensor_msgs::msg::LaserScan>(
            scan1_topic_, qos,
            [this](sensor_msgs::msg::LaserScan::SharedPtr msg) { s1_ = msg; });

        pub_ = create_publisher<sensor_msgs::msg::LaserScan>(output_topic_, qos);

        tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&MergeLaserScanNode::try_merge, this));

        RCLCPP_INFO(get_logger(), "%s + %s -> %s (frame=%s)",
            scan0_topic_.c_str(), scan1_topic_.c_str(),
            output_topic_.c_str(), target_frame_.c_str());
    }

private:
    static double yaw_of(const geometry_msgs::msg::Quaternion & q)
    {
        double t3 = 2.0 * (q.w * q.z + q.x * q.y);
        double t4 = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        return std::atan2(t3, t4);
    }

    using Points = std::vector<std::pair<double, double>>;

    static Points scan_to_points(
        const sensor_msgs::msg::LaserScan & scan,
        const geometry_msgs::msg::TransformStamped & tf)
    {
        double tx  = tf.transform.translation.x;
        double ty  = tf.transform.translation.y;
        double yaw = yaw_of(tf.transform.rotation);
        double cy  = std::cos(yaw);
        double sy  = std::sin(yaw);

        Points pts;
        pts.reserve(scan.ranges.size());
        double angle = scan.angle_min;
        for (float r : scan.ranges) {
            if (std::isfinite(r) && r > 0.0f) {
                double x = r * std::cos(angle);
                double y = r * std::sin(angle);
                pts.emplace_back(tx + x * cy - y * sy,
                                 ty + x * sy + y * cy);
            }
            angle += scan.angle_increment;
        }
        return pts;
    }

    void try_merge()
    {
        if (!s0_ || !s1_) return;

        geometry_msgs::msg::TransformStamped tf0, tf1;
        auto timeout = tf2::durationFromSec(tf_timeout_sec_);
        try {
            tf0 = tf_buffer_->lookupTransform(
                target_frame_, s0_->header.frame_id, tf2::TimePointZero, timeout);
            tf1 = tf_buffer_->lookupTransform(
                target_frame_, s1_->header.frame_id, tf2::TimePointZero, timeout);
        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "TF lookup failed: %s", ex.what());
            return;
        }

        sensor_msgs::msg::LaserScan out;
        out.header.frame_id = target_frame_;
        out.header.stamp    = (s0_->header.stamp.sec >= s1_->header.stamp.sec)
                              ? s0_->header.stamp : s1_->header.stamp;

        double inc0 = std::abs(s0_->angle_increment);
        double inc1 = std::abs(s1_->angle_increment);
        out.angle_min       = -M_PI;
        out.angle_max       =  M_PI;
        out.angle_increment = (inc0 > 0.0 && inc1 > 0.0) ? std::min(inc0, inc1) :
                              (inc0 > 0.0 ? inc0 : (inc1 > 0.0 ? inc1 : M_PI / 360.0));
        out.range_min       = std::min(s0_->range_min, s1_->range_min);
        out.range_max       = std::max(s0_->range_max, s1_->range_max);
        out.time_increment  = 0.0f;
        out.scan_time       = 0.0f;

        const int   N   = static_cast<int>(
            std::floor((out.angle_max - out.angle_min) / out.angle_increment)) + 1;
        const float INF = std::numeric_limits<float>::infinity();
        out.ranges.assign(N, INF);

        auto fill = [&](const Points & pts) {
            for (auto & [x, y] : pts) {
                double r = std::hypot(x, y);
                if (r < out.range_min || r > out.range_max) continue;
                int idx = static_cast<int>(
                    std::round((std::atan2(y, x) - out.angle_min) / out.angle_increment));
                if (idx >= 0 && idx < N && static_cast<float>(r) < out.ranges[idx])
                    out.ranges[idx] = static_cast<float>(r);
            }
        };

        fill(scan_to_points(*s0_, tf0));
        fill(scan_to_points(*s1_, tf1));

        pub_->publish(out);
    }

    std::string scan0_topic_, scan1_topic_, output_topic_, target_frame_;
    double      tf_timeout_sec_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub0_, sub1_;
    rclcpp::Publisher   <sensor_msgs::msg::LaserScan>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr                                 timer_;

    std::shared_ptr<tf2_ros::Buffer>             tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener>  tf_listener_;

    sensor_msgs::msg::LaserScan::SharedPtr s0_, s1_;
};


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MergeLaserScanNode>());
    rclcpp::shutdown();
    return 0;
}
