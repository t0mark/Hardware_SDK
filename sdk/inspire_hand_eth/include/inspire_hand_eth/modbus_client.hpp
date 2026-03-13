#ifndef INSPIRE_HAND_ETH__MODBUS_CLIENT_HPP_
#define INSPIRE_HAND_ETH__MODBUS_CLIENT_HPP_

#include <string>
#include <vector>
#include <memory>
#include <mutex>
#include <modbus/modbus.h>
#include <rclcpp/rclcpp.hpp>

namespace inspire_hand_eth
{

class ModbusClient
{
public:
    ModbusClient(const std::string & ip, int port, rclcpp::Logger logger);
    ~ModbusClient();

    // Connection management
    bool connect();
    void disconnect();
    bool isConnected() const;

    // Single register read/write
    bool readRegister(uint16_t address, int16_t & value);
    bool writeRegister(uint16_t address, uint16_t value);

    // Multiple registers read/write (contiguous)
    bool readRegisters(uint16_t address, int count, std::vector<int16_t> & values);
    bool writeRegisters(uint16_t address, const std::vector<uint16_t> & values);

    // Non-contiguous register read (for arrays like REG_ANGLE_ACT)
    template<std::size_t N>
    bool readNonContiguousRegisters(const std::array<uint16_t, N> & addresses,
                                     std::vector<int16_t> & values)
    {
        values.resize(N);
        for (std::size_t i = 0; i < N; ++i) {
            if (!readRegister(addresses[i], values[i])) {
                return false;
            }
        }
        return true;
    }

private:
    std::string ip_;
    int port_;
    modbus_t * ctx_;
    rclcpp::Logger logger_;
    mutable std::mutex mutex_;
};

}  // namespace inspire_hand_eth

#endif  // INSPIRE_HAND_ETH__MODBUS_CLIENT_HPP_
