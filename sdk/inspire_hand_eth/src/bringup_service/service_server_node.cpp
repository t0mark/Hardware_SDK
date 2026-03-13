#include <functional>
#include <memory>
#include <string>
#include <chrono>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "inspire_hand_eth/modbus_client.hpp"
#include "inspire_hand_eth/register_map.hpp"

// Service interfaces (26 services)
#include "service_interfaces/srv/getangleact.hpp"
#include "service_interfaces/srv/getangleset.hpp"
#include "service_interfaces/srv/getposact.hpp"
#include "service_interfaces/srv/getposset.hpp"
#include "service_interfaces/srv/getspeedset.hpp"
#include "service_interfaces/srv/getforceact.hpp"
#include "service_interfaces/srv/getforceset.hpp"
#include "service_interfaces/srv/getcurrentact.hpp"
#include "service_interfaces/srv/gettemp.hpp"
#include "service_interfaces/srv/geterror.hpp"
#include "service_interfaces/srv/getstatus.hpp"
#include "service_interfaces/srv/setangle.hpp"
#include "service_interfaces/srv/setpos.hpp"
#include "service_interfaces/srv/setspeed.hpp"
#include "service_interfaces/srv/setforce.hpp"
#include "service_interfaces/srv/setforceclb.hpp"
#include "service_interfaces/srv/setclearerror.hpp"
#include "service_interfaces/srv/setcurrentlimit.hpp"
#include "service_interfaces/srv/setdefaultforce.hpp"
#include "service_interfaces/srv/setdefaultspeed.hpp"
#include "service_interfaces/srv/setid.hpp"
#include "service_interfaces/srv/setreduratio.hpp"
#include "service_interfaces/srv/setresetpara.hpp"
#include "service_interfaces/srv/setsaveflash.hpp"
#include "service_interfaces/srv/setgestureno.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using namespace inspire_hand_eth;

class ServiceServerNode : public rclcpp::Node
{
public:
    ServiceServerNode()
    : Node("inspire_hand_service_server")
    {
        // Load config.yaml
        std::string pkg_share = ament_index_cpp::get_package_share_directory("inspire_hand_eth");
        std::string config_path = pkg_share + "/config/config.yaml";

        YAML::Node config = YAML::LoadFile(config_path);
        std::string ip = config["inspire_hand"]["modbus"]["ip"].as<std::string>();
        int port = config["inspire_hand"]["modbus"]["port"].as<int>();

        RCLCPP_INFO(this->get_logger(), "Config loaded: %s:%d", ip.c_str(), port);

        // Create Modbus client and connect
        modbus_client_ = std::make_shared<ModbusClient>(ip, port, this->get_logger());

        if (!modbus_client_->connect()) {
            RCLCPP_FATAL(this->get_logger(), "Failed to connect to Modbus TCP server");
            rclcpp::shutdown();
            return;
        }

        // Create all 26 services
        create_services();

        RCLCPP_INFO(this->get_logger(), "Service server started with 26 services");
    }

private:
    std::shared_ptr<ModbusClient> modbus_client_;

    // Service pointers (11 GET + 15 SET = 26)
    rclcpp::Service<service_interfaces::srv::Getangleact>::SharedPtr srv_getangleact_;
    rclcpp::Service<service_interfaces::srv::Getangleset>::SharedPtr srv_getangleset_;
    rclcpp::Service<service_interfaces::srv::Getposact>::SharedPtr srv_getposact_;
    rclcpp::Service<service_interfaces::srv::Getposset>::SharedPtr srv_getposset_;
    rclcpp::Service<service_interfaces::srv::Getspeedset>::SharedPtr srv_getspeedset_;
    rclcpp::Service<service_interfaces::srv::Getforceact>::SharedPtr srv_getforceact_;
    rclcpp::Service<service_interfaces::srv::Getforceset>::SharedPtr srv_getforceset_;
    rclcpp::Service<service_interfaces::srv::Getcurrentact>::SharedPtr srv_getcurrentact_;
    rclcpp::Service<service_interfaces::srv::Gettemp>::SharedPtr srv_gettemp_;
    rclcpp::Service<service_interfaces::srv::Geterror>::SharedPtr srv_geterror_;
    rclcpp::Service<service_interfaces::srv::Getstatus>::SharedPtr srv_getstatus_;

    rclcpp::Service<service_interfaces::srv::Setangle>::SharedPtr srv_setangle_;
    rclcpp::Service<service_interfaces::srv::Setpos>::SharedPtr srv_setpos_;
    rclcpp::Service<service_interfaces::srv::Setspeed>::SharedPtr srv_setspeed_;
    rclcpp::Service<service_interfaces::srv::Setforce>::SharedPtr srv_setforce_;
    rclcpp::Service<service_interfaces::srv::Setforceclb>::SharedPtr srv_setforceclb_;
    rclcpp::Service<service_interfaces::srv::Setclearerror>::SharedPtr srv_setclearerror_;
    rclcpp::Service<service_interfaces::srv::Setcurrentlimit>::SharedPtr srv_setcurrentlimit_;
    rclcpp::Service<service_interfaces::srv::Setdefaultforce>::SharedPtr srv_setdefaultforce_;
    rclcpp::Service<service_interfaces::srv::Setdefaultspeed>::SharedPtr srv_setdefaultspeed_;
    rclcpp::Service<service_interfaces::srv::Setid>::SharedPtr srv_setid_;
    rclcpp::Service<service_interfaces::srv::Setreduratio>::SharedPtr srv_setreduratio_;
    rclcpp::Service<service_interfaces::srv::Setresetpara>::SharedPtr srv_setresetpara_;
    rclcpp::Service<service_interfaces::srv::Setsaveflash>::SharedPtr srv_setsaveflash_;
    rclcpp::Service<service_interfaces::srv::Setgestureno>::SharedPtr srv_setgestureno_;

    void create_services()
    {
        // GET services (11)
        srv_getangleact_ = this->create_service<service_interfaces::srv::Getangleact>(
            "Getangleact", std::bind(&ServiceServerNode::cb_getangleact, this, _1, _2));
        srv_getangleset_ = this->create_service<service_interfaces::srv::Getangleset>(
            "Getangleset", std::bind(&ServiceServerNode::cb_getangleset, this, _1, _2));
        srv_getposact_ = this->create_service<service_interfaces::srv::Getposact>(
            "Getposact", std::bind(&ServiceServerNode::cb_getposact, this, _1, _2));
        srv_getposset_ = this->create_service<service_interfaces::srv::Getposset>(
            "Getposset", std::bind(&ServiceServerNode::cb_getposset, this, _1, _2));
        srv_getspeedset_ = this->create_service<service_interfaces::srv::Getspeedset>(
            "Getspeedset", std::bind(&ServiceServerNode::cb_getspeedset, this, _1, _2));
        srv_getforceact_ = this->create_service<service_interfaces::srv::Getforceact>(
            "Getforceact", std::bind(&ServiceServerNode::cb_getforceact, this, _1, _2));
        srv_getforceset_ = this->create_service<service_interfaces::srv::Getforceset>(
            "Getforceset", std::bind(&ServiceServerNode::cb_getforceset, this, _1, _2));
        srv_getcurrentact_ = this->create_service<service_interfaces::srv::Getcurrentact>(
            "Getcurrentact", std::bind(&ServiceServerNode::cb_getcurrentact, this, _1, _2));
        srv_gettemp_ = this->create_service<service_interfaces::srv::Gettemp>(
            "Gettemp", std::bind(&ServiceServerNode::cb_gettemp, this, _1, _2));
        srv_geterror_ = this->create_service<service_interfaces::srv::Geterror>(
            "Geterror", std::bind(&ServiceServerNode::cb_geterror, this, _1, _2));
        srv_getstatus_ = this->create_service<service_interfaces::srv::Getstatus>(
            "Getstatus", std::bind(&ServiceServerNode::cb_getstatus, this, _1, _2));

        // SET services (15)
        srv_setangle_ = this->create_service<service_interfaces::srv::Setangle>(
            "Setangle", std::bind(&ServiceServerNode::cb_setangle, this, _1, _2));
        srv_setpos_ = this->create_service<service_interfaces::srv::Setpos>(
            "Setpos", std::bind(&ServiceServerNode::cb_setpos, this, _1, _2));
        srv_setspeed_ = this->create_service<service_interfaces::srv::Setspeed>(
            "Setspeed", std::bind(&ServiceServerNode::cb_setspeed, this, _1, _2));
        srv_setforce_ = this->create_service<service_interfaces::srv::Setforce>(
            "Setforce", std::bind(&ServiceServerNode::cb_setforce, this, _1, _2));
        srv_setforceclb_ = this->create_service<service_interfaces::srv::Setforceclb>(
            "Setforceclb", std::bind(&ServiceServerNode::cb_setforceclb, this, _1, _2));
        srv_setclearerror_ = this->create_service<service_interfaces::srv::Setclearerror>(
            "Setclearerror", std::bind(&ServiceServerNode::cb_setclearerror, this, _1, _2));
        srv_setcurrentlimit_ = this->create_service<service_interfaces::srv::Setcurrentlimit>(
            "Setcurrentlimit", std::bind(&ServiceServerNode::cb_setcurrentlimit, this, _1, _2));
        srv_setdefaultforce_ = this->create_service<service_interfaces::srv::Setdefaultforce>(
            "Setdefaultforce", std::bind(&ServiceServerNode::cb_setdefaultforce, this, _1, _2));
        srv_setdefaultspeed_ = this->create_service<service_interfaces::srv::Setdefaultspeed>(
            "Setdefaultspeed", std::bind(&ServiceServerNode::cb_setdefaultspeed, this, _1, _2));
        srv_setid_ = this->create_service<service_interfaces::srv::Setid>(
            "Setid", std::bind(&ServiceServerNode::cb_setid, this, _1, _2));
        srv_setreduratio_ = this->create_service<service_interfaces::srv::Setreduratio>(
            "Setreduratio", std::bind(&ServiceServerNode::cb_setreduratio, this, _1, _2));
        srv_setresetpara_ = this->create_service<service_interfaces::srv::Setresetpara>(
            "Setresetpara", std::bind(&ServiceServerNode::cb_setresetpara, this, _1, _2));
        srv_setsaveflash_ = this->create_service<service_interfaces::srv::Setsaveflash>(
            "Setsaveflash", std::bind(&ServiceServerNode::cb_setsaveflash, this, _1, _2));
        srv_setgestureno_ = this->create_service<service_interfaces::srv::Setgestureno>(
            "Setgestureno", std::bind(&ServiceServerNode::cb_setgestureno, this, _1, _2));
    }

    // ===== GET Callbacks (11) =====

    void cb_getangleact(
        const service_interfaces::srv::Getangleact::Request::SharedPtr,
        const service_interfaces::srv::Getangleact::Response::SharedPtr response)
    {
        RCLCPP_INFO(this->get_logger(), "Getangleact called");
        std::vector<int16_t> values;
        if (modbus_client_->readNonContiguousRegisters(REG_ANGLE_ACT, values)) {
            for (int i = 0; i < NUM_FINGERS; ++i) {
                response->curangle[i] = values[i];
            }
        } else {
            for (int i = 0; i < NUM_FINGERS; ++i) {
                response->curangle[i] = -1;
            }
        }
    }

    void cb_getangleset(
        const service_interfaces::srv::Getangleset::Request::SharedPtr,
        const service_interfaces::srv::Getangleset::Response::SharedPtr response)
    {
        RCLCPP_INFO(this->get_logger(), "Getangleset called");
        std::vector<int16_t> values;
        if (modbus_client_->readNonContiguousRegisters(REG_ANGLE_SET, values)) {
            for (int i = 0; i < NUM_FINGERS; ++i) {
                response->setangle[i] = values[i];
            }
        } else {
            for (int i = 0; i < NUM_FINGERS; ++i) {
                response->setangle[i] = -1;
            }
        }
    }

    void cb_getposact(
        const service_interfaces::srv::Getposact::Request::SharedPtr,
        const service_interfaces::srv::Getposact::Response::SharedPtr response)
    {
        RCLCPP_INFO(this->get_logger(), "Getposact called");
        std::vector<int16_t> values;
        if (modbus_client_->readNonContiguousRegisters(REG_POS_ACT, values)) {
            for (int i = 0; i < NUM_FINGERS; ++i) {
                response->curpos[i] = values[i];
            }
        } else {
            for (int i = 0; i < NUM_FINGERS; ++i) {
                response->curpos[i] = -1;
            }
        }
    }

    void cb_getposset(
        const service_interfaces::srv::Getposset::Request::SharedPtr,
        const service_interfaces::srv::Getposset::Response::SharedPtr response)
    {
        RCLCPP_INFO(this->get_logger(), "Getposset called");
        std::vector<int16_t> values;
        if (modbus_client_->readNonContiguousRegisters(REG_POS_SET, values)) {
            for (int i = 0; i < NUM_FINGERS; ++i) {
                response->setpos[i] = values[i];
            }
        } else {
            for (int i = 0; i < NUM_FINGERS; ++i) {
                response->setpos[i] = -1;
            }
        }
    }

    void cb_getspeedset(
        const service_interfaces::srv::Getspeedset::Request::SharedPtr,
        const service_interfaces::srv::Getspeedset::Response::SharedPtr response)
    {
        RCLCPP_INFO(this->get_logger(), "Getspeedset called");
        std::vector<int16_t> values;
        if (modbus_client_->readNonContiguousRegisters(REG_SPEED_SET, values)) {
            for (int i = 0; i < NUM_FINGERS; ++i) {
                response->curspeedset[i] = values[i];
            }
        } else {
            for (int i = 0; i < NUM_FINGERS; ++i) {
                response->curspeedset[i] = -1;
            }
        }
    }

    void cb_getforceact(
        const service_interfaces::srv::Getforceact::Request::SharedPtr,
        const service_interfaces::srv::Getforceact::Response::SharedPtr response)
    {
        RCLCPP_INFO(this->get_logger(), "Getforceact called");
        std::vector<int16_t> values;
        if (modbus_client_->readNonContiguousRegisters(REG_FORCE_ACT, values)) {
            for (int i = 0; i < NUM_FINGERS; ++i) {
                response->curforce[i] = values[i];
            }
        } else {
            for (int i = 0; i < NUM_FINGERS; ++i) {
                response->curforce[i] = -1;
            }
        }
    }

    void cb_getforceset(
        const service_interfaces::srv::Getforceset::Request::SharedPtr,
        const service_interfaces::srv::Getforceset::Response::SharedPtr response)
    {
        RCLCPP_INFO(this->get_logger(), "Getforceset called");
        std::vector<int16_t> values;
        if (modbus_client_->readNonContiguousRegisters(REG_FORCE_SET, values)) {
            for (int i = 0; i < NUM_FINGERS; ++i) {
                response->setforce[i] = values[i];
            }
        } else {
            for (int i = 0; i < NUM_FINGERS; ++i) {
                response->setforce[i] = -1;
            }
        }
    }

    void cb_getcurrentact(
        const service_interfaces::srv::Getcurrentact::Request::SharedPtr,
        const service_interfaces::srv::Getcurrentact::Response::SharedPtr response)
    {
        RCLCPP_INFO(this->get_logger(), "Getcurrentact called");
        std::vector<int16_t> values;
        if (modbus_client_->readNonContiguousRegisters(REG_CURRENT_ACT, values)) {
            for (int i = 0; i < NUM_FINGERS; ++i) {
                response->current[i] = values[i];
            }
        } else {
            for (int i = 0; i < NUM_FINGERS; ++i) {
                response->current[i] = -1;
            }
        }
    }

    void cb_gettemp(
        const service_interfaces::srv::Gettemp::Request::SharedPtr,
        const service_interfaces::srv::Gettemp::Response::SharedPtr response)
    {
        RCLCPP_INFO(this->get_logger(), "Gettemp called");
        std::vector<int16_t> values;
        if (modbus_client_->readRegisters(REG_TEMP_START, 3, values)) {
            // Each register contains 2 temperature values (low byte, high byte)
            for (int i = 0; i < 3; ++i) {
                response->tempvalue[i * 2] = values[i] & 0xFF;
                response->tempvalue[i * 2 + 1] = (values[i] >> 8) & 0xFF;
            }
            response->success = true;
        } else {
            response->success = false;
        }
    }

    void cb_geterror(
        const service_interfaces::srv::Geterror::Request::SharedPtr request,
        const service_interfaces::srv::Geterror::Response::SharedPtr response)
    {
        RCLCPP_INFO(this->get_logger(), "Geterror called");
        if (request->status == "get_error") {
            std::vector<int16_t> values;
            if (modbus_client_->readRegisters(REG_ERROR_START, 3, values)) {
                for (int i = 0; i < 3; ++i) {
                    response->errorvalue[i] = values[i];
                }
                response->success = true;
            } else {
                response->success = false;
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "Unsupported status: %s", request->status.c_str());
            response->success = false;
        }
    }

    void cb_getstatus(
        const service_interfaces::srv::Getstatus::Request::SharedPtr,
        const service_interfaces::srv::Getstatus::Response::SharedPtr response)
    {
        RCLCPP_INFO(this->get_logger(), "Getstatus called");
        std::vector<int16_t> values;
        if (modbus_client_->readNonContiguousRegisters(REG_STATUS, values)) {
            for (int i = 0; i < NUM_FINGERS; ++i) {
                response->statusvalue[i] = values[i];
            }
        } else {
            for (int i = 0; i < NUM_FINGERS; ++i) {
                response->statusvalue[i] = -1;
            }
        }
    }

    // ===== SET Callbacks (15) =====

    void cb_setangle(
        const service_interfaces::srv::Setangle::Request::SharedPtr request,
        const service_interfaces::srv::Setangle::Response::SharedPtr response)
    {
        RCLCPP_INFO(this->get_logger(), "Setangle called");

        if ((request->angle0 >= -1 && request->angle0 <= 1000) &&
            (request->angle1 >= -1 && request->angle1 <= 1000) &&
            (request->angle2 >= -1 && request->angle2 <= 1000) &&
            (request->angle3 >= -1 && request->angle3 <= 1000) &&
            (request->angle4 >= -1 && request->angle4 <= 1000) &&
            (request->angle5 >= -1 && request->angle5 <= 1000))
        {
            std::vector<uint16_t> angles = {
                static_cast<uint16_t>(request->angle0),
                static_cast<uint16_t>(request->angle1),
                static_cast<uint16_t>(request->angle2),
                static_cast<uint16_t>(request->angle3),
                static_cast<uint16_t>(request->angle4),
                static_cast<uint16_t>(request->angle5)
            };
            response->angle_accepted = modbus_client_->writeRegisters(REG_ANGLE_SET[0], angles);
        } else {
            RCLCPP_WARN(this->get_logger(), "Angle values out of range (-1 to 1000)");
            response->angle_accepted = false;
        }
    }

    void cb_setpos(
        const service_interfaces::srv::Setpos::Request::SharedPtr request,
        const service_interfaces::srv::Setpos::Response::SharedPtr response)
    {
        RCLCPP_INFO(this->get_logger(), "Setpos called");

        if ((request->pos0 >= 0 && request->pos0 <= 2000) &&
            (request->pos1 >= 0 && request->pos1 <= 2000) &&
            (request->pos2 >= 0 && request->pos2 <= 2000) &&
            (request->pos3 >= 0 && request->pos3 <= 2000) &&
            (request->pos4 >= 0 && request->pos4 <= 2000) &&
            (request->pos5 >= 0 && request->pos5 <= 2000))
        {
            std::vector<uint16_t> positions = {
                static_cast<uint16_t>(request->pos0),
                static_cast<uint16_t>(request->pos1),
                static_cast<uint16_t>(request->pos2),
                static_cast<uint16_t>(request->pos3),
                static_cast<uint16_t>(request->pos4),
                static_cast<uint16_t>(request->pos5)
            };
            response->pos_accepted = modbus_client_->writeRegisters(REG_POS_SET[0], positions);
        } else {
            RCLCPP_WARN(this->get_logger(), "Position values out of range (0 to 2000)");
            response->pos_accepted = false;
        }
    }

    void cb_setspeed(
        const service_interfaces::srv::Setspeed::Request::SharedPtr request,
        const service_interfaces::srv::Setspeed::Response::SharedPtr response)
    {
        RCLCPP_INFO(this->get_logger(), "Setspeed called");

        if ((request->speed0 >= 0 && request->speed0 <= 1000) &&
            (request->speed1 >= 0 && request->speed1 <= 1000) &&
            (request->speed2 >= 0 && request->speed2 <= 1000) &&
            (request->speed3 >= 0 && request->speed3 <= 1000) &&
            (request->speed4 >= 0 && request->speed4 <= 1000) &&
            (request->speed5 >= 0 && request->speed5 <= 1000))
        {
            std::vector<uint16_t> speeds = {
                static_cast<uint16_t>(request->speed0),
                static_cast<uint16_t>(request->speed1),
                static_cast<uint16_t>(request->speed2),
                static_cast<uint16_t>(request->speed3),
                static_cast<uint16_t>(request->speed4),
                static_cast<uint16_t>(request->speed5)
            };
            response->speed_accepted = modbus_client_->writeRegisters(REG_SPEED_SET[0], speeds);
        } else {
            RCLCPP_WARN(this->get_logger(), "Speed values out of range (0 to 1000)");
            response->speed_accepted = false;
        }
    }

    void cb_setforce(
        const service_interfaces::srv::Setforce::Request::SharedPtr request,
        const service_interfaces::srv::Setforce::Response::SharedPtr response)
    {
        RCLCPP_INFO(this->get_logger(), "Setforce called");

        if ((request->force0 >= 0 && request->force0 <= 3000) &&
            (request->force1 >= 0 && request->force1 <= 3000) &&
            (request->force2 >= 0 && request->force2 <= 3000) &&
            (request->force3 >= 0 && request->force3 <= 3000) &&
            (request->force4 >= 0 && request->force4 <= 3000) &&
            (request->force5 >= 0 && request->force5 <= 3000))
        {
            std::vector<uint16_t> forces = {
                static_cast<uint16_t>(request->force0),
                static_cast<uint16_t>(request->force1),
                static_cast<uint16_t>(request->force2),
                static_cast<uint16_t>(request->force3),
                static_cast<uint16_t>(request->force4),
                static_cast<uint16_t>(request->force5)
            };
            response->force_accepted = modbus_client_->writeRegisters(REG_FORCE_SET[0], forces);
        } else {
            RCLCPP_WARN(this->get_logger(), "Force values out of range (0 to 3000)");
            response->force_accepted = false;
        }
    }

    void cb_setforceclb(
        const service_interfaces::srv::Setforceclb::Request::SharedPtr,
        const service_interfaces::srv::Setforceclb::Response::SharedPtr response)
    {
        RCLCPP_INFO(this->get_logger(), "Setforceclb called");

        // Write 1000 to angle registers for calibration
        std::vector<uint16_t> calib_values(6, 1000);
        if (!modbus_client_->writeRegisters(REG_ANGLE_SET[0], calib_values)) {
            response->setforce_clb_accepted = false;
            return;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        // Write 1 to force calibration register
        response->setforce_clb_accepted = modbus_client_->writeRegister(REG_FORCE_CLB, 1);
    }

    void cb_setclearerror(
        const service_interfaces::srv::Setclearerror::Request::SharedPtr,
        const service_interfaces::srv::Setclearerror::Response::SharedPtr response)
    {
        RCLCPP_INFO(this->get_logger(), "Setclearerror called");
        int16_t value;
        response->setclear_error_accepted = modbus_client_->readRegister(REG_CLEAR_ERROR, value);
    }

    void cb_setcurrentlimit(
        const service_interfaces::srv::Setcurrentlimit::Request::SharedPtr request,
        const service_interfaces::srv::Setcurrentlimit::Response::SharedPtr response)
    {
        RCLCPP_INFO(this->get_logger(), "Setcurrentlimit called");

        if ((request->current0 >= 0 && request->current0 <= 1500) &&
            (request->current1 >= 0 && request->current1 <= 1500) &&
            (request->current2 >= 0 && request->current2 <= 1500) &&
            (request->current3 >= 0 && request->current3 <= 1500) &&
            (request->current4 >= 0 && request->current4 <= 1500) &&
            (request->current5 >= 0 && request->current5 <= 1500))
        {
            std::vector<uint16_t> currents = {
                static_cast<uint16_t>(request->current0),
                static_cast<uint16_t>(request->current1),
                static_cast<uint16_t>(request->current2),
                static_cast<uint16_t>(request->current3),
                static_cast<uint16_t>(request->current4),
                static_cast<uint16_t>(request->current5)
            };
            response->current_limit_accepted = modbus_client_->writeRegisters(
                REG_CURRENT_LIMIT_START, currents);
        } else {
            RCLCPP_WARN(this->get_logger(), "Current limit values out of range (0 to 1500)");
            response->current_limit_accepted = false;
        }
    }

    void cb_setdefaultforce(
        const service_interfaces::srv::Setdefaultforce::Request::SharedPtr request,
        const service_interfaces::srv::Setdefaultforce::Response::SharedPtr response)
    {
        RCLCPP_INFO(this->get_logger(), "Setdefaultforce called");

        if ((request->force0 >= 0 && request->force0 <= 3000) &&
            (request->force1 >= 0 && request->force1 <= 3000) &&
            (request->force2 >= 0 && request->force2 <= 3000) &&
            (request->force3 >= 0 && request->force3 <= 3000) &&
            (request->force4 >= 0 && request->force4 <= 3000) &&
            (request->force5 >= 0 && request->force5 <= 3000))
        {
            std::vector<uint16_t> forces = {
                static_cast<uint16_t>(request->force0),
                static_cast<uint16_t>(request->force1),
                static_cast<uint16_t>(request->force2),
                static_cast<uint16_t>(request->force3),
                static_cast<uint16_t>(request->force4),
                static_cast<uint16_t>(request->force5)
            };
            if (modbus_client_->writeRegisters(REG_DEFAULT_FORCE_START, forces)) {
                // Save to flash
                response->default_force_accepted = modbus_client_->writeRegister(REG_SAVE_FLASH, 1);
            } else {
                response->default_force_accepted = false;
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "Default force values out of range (0 to 3000)");
            response->default_force_accepted = false;
        }
    }

    void cb_setdefaultspeed(
        const service_interfaces::srv::Setdefaultspeed::Request::SharedPtr request,
        const service_interfaces::srv::Setdefaultspeed::Response::SharedPtr response)
    {
        RCLCPP_INFO(this->get_logger(), "Setdefaultspeed called");

        if ((request->speed0 >= 0 && request->speed0 <= 1000) &&
            (request->speed1 >= 0 && request->speed1 <= 1000) &&
            (request->speed2 >= 0 && request->speed2 <= 1000) &&
            (request->speed3 >= 0 && request->speed3 <= 1000) &&
            (request->speed4 >= 0 && request->speed4 <= 1000) &&
            (request->speed5 >= 0 && request->speed5 <= 1000))
        {
            std::vector<uint16_t> speeds = {
                static_cast<uint16_t>(request->speed0),
                static_cast<uint16_t>(request->speed1),
                static_cast<uint16_t>(request->speed2),
                static_cast<uint16_t>(request->speed3),
                static_cast<uint16_t>(request->speed4),
                static_cast<uint16_t>(request->speed5)
            };
            if (modbus_client_->writeRegisters(REG_DEFAULT_SPEED_START, speeds)) {
                // Save to flash
                response->default_speed_accepted = modbus_client_->writeRegister(REG_SAVE_FLASH, 1);
            } else {
                response->default_speed_accepted = false;
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "Default speed values out of range (0 to 1000)");
            response->default_speed_accepted = false;
        }
    }

    void cb_setid(
        const service_interfaces::srv::Setid::Request::SharedPtr request,
        const service_interfaces::srv::Setid::Response::SharedPtr response)
    {
        RCLCPP_INFO(this->get_logger(), "Setid called");

        if (request->id >= 1 && request->id <= 254) {
            if (modbus_client_->writeRegister(REG_ID, static_cast<uint16_t>(request->id))) {
                // Verify
                int16_t read_value;
                if (modbus_client_->readRegister(REG_ID, read_value)) {
                    response->idgrab = (read_value == request->id);
                } else {
                    response->idgrab = false;
                }
            } else {
                response->idgrab = false;
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "ID out of range (1 to 254)");
            response->idgrab = false;
        }
    }

    void cb_setreduratio(
        const service_interfaces::srv::Setreduratio::Request::SharedPtr request,
        const service_interfaces::srv::Setreduratio::Response::SharedPtr response)
    {
        RCLCPP_INFO(this->get_logger(), "Setreduratio called");

        if (request->redu_ratio >= 0 && request->redu_ratio <= 4) {
            response->redu_ratiograb = modbus_client_->writeRegister(
                REG_REDU_RATIO, static_cast<uint16_t>(request->redu_ratio));
            response->success = response->redu_ratiograb;
        } else {
            RCLCPP_WARN(this->get_logger(), "Reduction ratio out of range (0 to 4)");
            response->redu_ratiograb = false;
            response->success = false;
        }
    }

    void cb_setresetpara(
        const service_interfaces::srv::Setresetpara::Request::SharedPtr,
        const service_interfaces::srv::Setresetpara::Response::SharedPtr response)
    {
        RCLCPP_INFO(this->get_logger(), "Setresetpara called");
        response->setreset_para_accepted = modbus_client_->writeRegister(REG_RESET_PARA, 1);
    }

    void cb_setsaveflash(
        const service_interfaces::srv::Setsaveflash::Request::SharedPtr,
        const service_interfaces::srv::Setsaveflash::Response::SharedPtr response)
    {
        RCLCPP_INFO(this->get_logger(), "Setsaveflash called");
        response->setsave_flash_accepted = modbus_client_->writeRegister(REG_SAVE_FLASH, 1);
    }

    void cb_setgestureno(
        const service_interfaces::srv::Setgestureno::Request::SharedPtr request,
        const service_interfaces::srv::Setgestureno::Response::SharedPtr response)
    {
        RCLCPP_INFO(this->get_logger(), "Setgestureno called");

        if (request->gesture_no >= 0 && request->gesture_no <= 13) {
            if (modbus_client_->writeRegister(REG_GESTURE_NO,
                                               static_cast<uint16_t>(request->gesture_no))) {
                // Execute gesture
                response->gesture_nograb = modbus_client_->writeRegister(REG_GESTURE_ACTION, 1);
            } else {
                response->gesture_nograb = false;
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "Gesture number out of range (0 to 13)");
            response->gesture_nograb = false;
        }
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ServiceServerNode>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
