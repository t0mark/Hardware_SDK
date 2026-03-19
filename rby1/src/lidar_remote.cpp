#include <curl/curl.h>
#include "lidar/lidar_remote.h"

static size_t dummy_callback(void* /*buffer*/, size_t size, size_t nmemb, void* /*userp*/)
{
    return size * nmemb;
}

int sensor_config(const std::string& sensor_ipaddr,
                  const std::string& parameter,
                  const std::string& value,
                  const rclcpp::Logger& logger)
{
    std::string url = "http://" + sensor_ipaddr + parameter;
    RCLCPP_INFO(logger, "sensor_config: PUT %s = %s", url.c_str(), value.c_str());

    CURL* curl = curl_easy_init();
    if (!curl) {
        RCLCPP_WARN(logger, "sensor_config: curl_easy_init failed");
        return -1;
    }

    curl_easy_setopt(curl, CURLOPT_URL,            url.c_str());
    curl_easy_setopt(curl, CURLOPT_TIMEOUT,        3L);
    curl_easy_setopt(curl, CURLOPT_CUSTOMREQUEST,  "PUT");
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS,     value.c_str());
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION,  dummy_callback);

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
