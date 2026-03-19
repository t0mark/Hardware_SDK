#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>
#include <cmath>
#include <limits>
#include <thread>
#include <vector>
#include <string>

#include "lidar/data_type.h"
#include "lidar/lidar_remote.h"

#define DEG2RAD(x) ((x) * M_PI / 180.0f)

class LidarNode : public rclcpp::Node
{
public:
    LidarNode()
    : Node("lidar_node")
    {
        // ── 파라미터 선언 ──────────────────────────────────────────────────
        declare_parameter<std::string>("sensorip",         "192.168.30.10");
        declare_parameter<std::string>("hostip",           "0.0.0.0");
        declare_parameter<std::string>("port",             "2368");
        declare_parameter<std::string>("frame_id",         "lidar_left");
        declare_parameter<std::string>("output_topic",     "scan_left");
        declare_parameter<bool>       ("inverted",         false);
        declare_parameter<int>        ("angle_offset",     0);
        declare_parameter<bool>  ("configure_sensor", true);
        declare_parameter<int64_t>("scanfreq",         30);
        declare_parameter<bool>  ("laser_enable",      true);
        declare_parameter<int64_t>("scan_range_start", 45);
        declare_parameter<int64_t>("scan_range_stop",  315);

        get_parameter("sensorip",         sensorip_);
        get_parameter("hostip",           hostip_);
        get_parameter("port",             port_);
        get_parameter("frame_id",         frame_id_);
        get_parameter("output_topic",     output_topic_);
        get_parameter("inverted",         inverted_);
        get_parameter("angle_offset",     angle_offset_);
        get_parameter("configure_sensor", configure_sensor_);
        get_parameter("scanfreq",         scanfreq_);
        get_parameter("laser_enable",     laser_enable_);
        get_parameter("scan_range_start", scan_range_start_);
        get_parameter("scan_range_stop",  scan_range_stop_);

        RCLCPP_INFO(get_logger(), "scanfreq:    %ld Hz", scanfreq_);
        RCLCPP_INFO(get_logger(), "laser_enable:%s", laser_enable_ ? "true" : "false");
        RCLCPP_INFO(get_logger(), "scan_range:  [%ld, %ld] deg", scan_range_start_, scan_range_stop_);

        RCLCPP_INFO(get_logger(), "sensorip:    %s", sensorip_.c_str());
        RCLCPP_INFO(get_logger(), "port:        %s", port_.c_str());
        RCLCPP_INFO(get_logger(), "frame_id:    %s", frame_id_.c_str());
        RCLCPP_INFO(get_logger(), "topic:       %s", output_topic_.c_str());
        RCLCPP_INFO(get_logger(), "inverted:    %s", inverted_ ? "true" : "false");

        // ── Publisher ─────────────────────────────────────────────────────
        scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(output_topic_, 1000);

        // ── HTTP 센서 설정 (configure_sensor == true) ─────────────────────
        if (configure_sensor_) {
            RCLCPP_INFO(get_logger(), "Configuring LiDAR via HTTP ...");
            sensor_config(sensorip_, "/api/v1/sensor/scanfreq",         std::to_string(scanfreq_),         get_logger());
            sensor_config(sensorip_, "/api/v1/sensor/laser_enable",     laser_enable_ ? "true" : "false",  get_logger());
            sensor_config(sensorip_, "/api/v1/sensor/scan_range/start", std::to_string(scan_range_start_), get_logger());
            sensor_config(sensorip_, "/api/v1/sensor/scan_range/stop",  std::to_string(scan_range_stop_),  get_logger());
        }

        // ── UDP 소켓 생성 ─────────────────────────────────────────────────
        if (!create_socket()) {
            RCLCPP_ERROR(get_logger(), "Failed to create UDP socket — LiDAR data will not be received");
            return;
        }

        // ── 스캔 루프 스레드 시작 ─────────────────────────────────────────
        scan_thread_ = std::thread(&LidarNode::scan_loop, this);
    }

    ~LidarNode()
    {
        if (sockfd_ >= 0) {
            close(sockfd_);
        }
        if (scan_thread_.joinable()) {
            scan_thread_.join();
        }
    }

private:
    // ── 소켓 생성 ──────────────────────────────────────────────────────────
    bool create_socket()
    {
        sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (sockfd_ < 0) {
            RCLCPP_ERROR(get_logger(), "socket() failed: %s", strerror(errno));
            return false;
        }

        // SO_RCVTIMEO: 1초 타임아웃 → rclcpp::ok() 체크 가능
        struct timeval tv;
        tv.tv_sec  = 1;
        tv.tv_usec = 0;
        setsockopt(sockfd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

        memset(&ser_addr_, 0, sizeof(ser_addr_));
        ser_addr_.sin_family      = AF_INET;
        ser_addr_.sin_addr.s_addr = inet_addr(hostip_.c_str());
        ser_addr_.sin_port        = htons(static_cast<uint16_t>(std::stoi(port_)));

        if (bind(sockfd_, reinterpret_cast<struct sockaddr*>(&ser_addr_), sizeof(ser_addr_)) < 0) {
            RCLCPP_ERROR(get_logger(), "bind() failed on %s:%s — %s", hostip_.c_str(), port_.c_str(), strerror(errno));
            close(sockfd_);
            sockfd_ = -1;
            return false;
        }

        RCLCPP_INFO(get_logger(), "UDP socket bound to %s:%s", hostip_.c_str(), port_.c_str());
        return true;
    }

    // ── 스캔 루프 (별도 스레드) ────────────────────────────────────────────
    void scan_loop()
    {
        const double inf = std::numeric_limits<double>::infinity();

        MSOP_Packet    pkt;
        struct sockaddr_in client_addr;
        socklen_t      addr_len = sizeof(client_addr);

        std::vector<bm_response_scan_t> scan_vec;
        rclcpp::Time scan_begin, scan_end;
        int  i = 0, j = 12;
        int  resolution = 25;
        bool scan_ready = false;

        RCLCPP_INFO(get_logger(), "Scan loop started");

        while (rclcpp::ok()) {
            // ── 패킷 수신 ────────────────────────────────────────────────
            if (!scan_ready) {
                while (rclcpp::ok()) {
                    if (j == 12) {
                        ssize_t n = recvfrom(sockfd_, &pkt, sizeof(pkt), 0,
                                             reinterpret_cast<struct sockaddr*>(&client_addr),
                                             &addr_len);
                        if (n < 0) {
                            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                                // 타임아웃: rclcpp::ok() 재확인
                                break;
                            }
                            RCLCPP_WARN(get_logger(), "recvfrom error: %s", strerror(errno));
                            break;
                        }

                        // 새 스캔 시작 감지 (첫 블록 azimuth == 0)
                        if (pkt.BlockID[0].Azimuth == 0) {
                            scan_end   = scan_begin;
                            scan_begin = rclcpp::Clock().now();
                        }

                        // 각도 해상도 계산
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

                            if (resp.angle == 0) {
                                if (!scan_vec.empty()) {
                                    scan_ready = true;
                                    if (scan_vec.size() < 1200) {
                                        j = 12;  // 다음 패킷부터 새 스캔
                                    }
                                    break;
                                }
                            }
                            resp.dist = pkt.BlockID[j].Result[i].Dist_1;
                            resp.rssi = pkt.BlockID[j].Result[i].RSSI_1;
                            scan_vec.push_back(resp);
                        }
                        if (scan_ready) break;
                    }
                    if (scan_ready) break;
                }
            }

            // ── LaserScan 발행 ────────────────────────────────────────────
            if (scan_ready) {
                const uint32_t num_readings = static_cast<uint32_t>(scan_vec.size());
                float duration = static_cast<float>((scan_begin - scan_end).seconds());

                sensor_msgs::msg::LaserScan scan;
                scan.header.stamp    = scan_begin;
                scan.header.frame_id = frame_id_;
                scan.angle_min       = DEG2RAD(-180 + angle_offset_);
                scan.angle_max       = DEG2RAD( 180 + angle_offset_);
                scan.angle_increment = 2.0f * static_cast<float>(M_PI) / num_readings;
                scan.scan_time       = duration;
                scan.time_increment  = duration / static_cast<float>(num_readings) / 2.0f;
                scan.range_min       = 0.0f;
                scan.range_max       = 100.0f;
                scan.ranges.resize(num_readings);
                scan.intensities.resize(num_readings);

                for (uint32_t idx = 0; idx < num_readings; ++idx) {
                    uint32_t dst_idx = inverted_ ? (num_readings - idx - 1) : idx;
                    float    range   = static_cast<float>(scan_vec[idx].dist) / 1000.0f;
                    if (range == 0.0f) {
                        scan.ranges[dst_idx]      = static_cast<float>(inf);
                        scan.intensities[dst_idx] = 0.0f;
                    } else {
                        scan.ranges[dst_idx]      = range;
                        scan.intensities[dst_idx] = static_cast<float>(scan_vec[idx].rssi);
                    }
                }

                scan_pub_->publish(scan);
                RCLCPP_DEBUG(get_logger(), "Published %s (%u pts)", output_topic_.c_str(), num_readings);

                scan_vec.clear();
                scan_ready = false;
            }
        }

        RCLCPP_INFO(get_logger(), "Scan loop exited");
    }

    // ── 멤버 변수 ──────────────────────────────────────────────────────────
    std::string sensorip_, hostip_, port_, frame_id_, output_topic_;
    int64_t     scanfreq_, scan_range_start_, scan_range_stop_;
    bool        laser_enable_;
    bool        inverted_;
    int         angle_offset_;
    bool        configure_sensor_;

    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;

    int                sockfd_ = -1;
    struct sockaddr_in ser_addr_;

    std::thread scan_thread_;
};


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarNode>());
    rclcpp::shutdown();
    return 0;
}
