#include "inspire_hand_eth/modbus_client.hpp"

namespace inspire_hand_eth
{

ModbusClient::ModbusClient(const std::string & ip, int port, rclcpp::Logger logger)
: ip_(ip), port_(port), ctx_(nullptr), logger_(logger)
{
}

ModbusClient::~ModbusClient()
{
    disconnect();
}

bool ModbusClient::connect()
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (ctx_) {
        modbus_close(ctx_);
        modbus_free(ctx_);
    }

    ctx_ = modbus_new_tcp(ip_.c_str(), port_);
    if (!ctx_) {
        RCLCPP_ERROR(logger_, "Failed to create Modbus context");
        return false;
    }

    if (modbus_connect(ctx_) == -1) {
        RCLCPP_ERROR(logger_, "Failed to connect to %s:%d - %s",
                     ip_.c_str(), port_, modbus_strerror(errno));
        modbus_free(ctx_);
        ctx_ = nullptr;
        return false;
    }

    RCLCPP_INFO(logger_, "Connected to Modbus TCP server at %s:%d", ip_.c_str(), port_);
    return true;
}

void ModbusClient::disconnect()
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (ctx_) {
        modbus_close(ctx_);
        modbus_free(ctx_);
        ctx_ = nullptr;
        RCLCPP_INFO(logger_, "Disconnected from Modbus TCP server");
    }
}

bool ModbusClient::isConnected() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return ctx_ != nullptr;
}

bool ModbusClient::readRegister(uint16_t address, int16_t & value)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!ctx_) {
        RCLCPP_ERROR(logger_, "Not connected");
        return false;
    }

    uint16_t tab_reg[1];
    int rc = modbus_read_registers(ctx_, address, 1, tab_reg);
    if (rc == -1) {
        RCLCPP_ERROR(logger_, "Read register %d failed: %s", address, modbus_strerror(errno));
        return false;
    }
    value = static_cast<int16_t>(tab_reg[0]);
    return true;
}

bool ModbusClient::writeRegister(uint16_t address, uint16_t value)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!ctx_) {
        RCLCPP_ERROR(logger_, "Not connected");
        return false;
    }

    int rc = modbus_write_register(ctx_, address, value);
    if (rc == -1) {
        RCLCPP_ERROR(logger_, "Write register %d failed: %s", address, modbus_strerror(errno));
        return false;
    }
    return true;
}

bool ModbusClient::readRegisters(uint16_t address, int count, std::vector<int16_t> & values)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!ctx_) {
        RCLCPP_ERROR(logger_, "Not connected");
        return false;
    }

    std::vector<uint16_t> tab_reg(count);
    int rc = modbus_read_registers(ctx_, address, count, tab_reg.data());
    if (rc == -1) {
        RCLCPP_ERROR(logger_, "Read registers from %d (count=%d) failed: %s",
                     address, count, modbus_strerror(errno));
        return false;
    }

    values.resize(count);
    for (int i = 0; i < count; ++i) {
        values[i] = static_cast<int16_t>(tab_reg[i]);
    }
    return true;
}

bool ModbusClient::writeRegisters(uint16_t address, const std::vector<uint16_t> & values)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!ctx_) {
        RCLCPP_ERROR(logger_, "Not connected");
        return false;
    }

    int rc = modbus_write_registers(ctx_, address, values.size(), values.data());
    if (rc == -1) {
        RCLCPP_ERROR(logger_, "Write registers to %d failed: %s", address, modbus_strerror(errno));
        return false;
    }
    return true;
}

}  // namespace inspire_hand_eth
