#ifndef INSPIRE_HAND_ETH__REGISTER_MAP_HPP_
#define INSPIRE_HAND_ETH__REGISTER_MAP_HPP_

#include <array>
#include <cstdint>

namespace inspire_hand_eth
{

constexpr int NUM_FINGERS = 6;

// ===== GET Registers (Read) =====

// Angle Actual (current angle)
constexpr std::array<uint16_t, NUM_FINGERS> REG_ANGLE_ACT = {
    1546, 1548, 1550, 1552, 1554, 1556
};

// Angle Set (target angle)
constexpr std::array<uint16_t, NUM_FINGERS> REG_ANGLE_SET = {
    1486, 1488, 1490, 1492, 1494, 1496
};

// Position Actual (current position)
constexpr std::array<uint16_t, NUM_FINGERS> REG_POS_ACT = {
    1534, 1536, 1538, 1540, 1542, 1544
};

// Position Set (target position)
constexpr std::array<uint16_t, NUM_FINGERS> REG_POS_SET = {
    1474, 1476, 1478, 1480, 1482, 1484
};

// Speed Set (target speed)
constexpr std::array<uint16_t, NUM_FINGERS> REG_SPEED_SET = {
    1522, 1524, 1526, 1528, 1530, 1532
};

// Force Actual (current force)
constexpr std::array<uint16_t, NUM_FINGERS> REG_FORCE_ACT = {
    1582, 1584, 1586, 1588, 1590, 1592
};

// Force Set (target force)
constexpr std::array<uint16_t, NUM_FINGERS> REG_FORCE_SET = {
    1498, 1500, 1502, 1504, 1506, 1508
};

// Current Actual (current in mA)
constexpr std::array<uint16_t, NUM_FINGERS> REG_CURRENT_ACT = {
    1594, 1596, 1598, 1600, 1602, 1604
};

// Temperature (6 consecutive registers from 1618)
constexpr uint16_t REG_TEMP_START = 1618;

// Error (6 consecutive registers from 1606)
constexpr uint16_t REG_ERROR_START = 1606;

// Status
constexpr std::array<uint16_t, NUM_FINGERS> REG_STATUS = {
    1612, 1614, 1616, 1618, 1620, 1622
};

// ===== SET Registers (Write) =====

// Default Speed (6 consecutive registers from 1032)
constexpr uint16_t REG_DEFAULT_SPEED_START = 1032;

// Default Force (6 consecutive registers from 1044)
constexpr uint16_t REG_DEFAULT_FORCE_START = 1044;

// Current Limit (6 consecutive registers from 1020)
constexpr uint16_t REG_CURRENT_LIMIT_START = 1020;

// ID
constexpr uint16_t REG_ID = 1000;

// Reduction Ratio
constexpr uint16_t REG_REDU_RATIO = 1002;

// Clear Error
constexpr uint16_t REG_CLEAR_ERROR = 1004;

// Save Flash
constexpr uint16_t REG_SAVE_FLASH = 1005;

// Reset Parameter
constexpr uint16_t REG_RESET_PARA = 1006;

// Force Calibration
constexpr uint16_t REG_FORCE_CLB = 1009;

// Gesture Number
constexpr uint16_t REG_GESTURE_NO = 2320;
constexpr uint16_t REG_GESTURE_ACTION = 2322;

// ===== Touch Sensor Registers =====

struct TouchRange {
    uint16_t start;
    uint16_t end;
};

constexpr TouchRange TOUCH_PINKY  = {3000, 3369};
constexpr TouchRange TOUCH_RING   = {3370, 3739};
constexpr TouchRange TOUCH_MIDDLE = {3740, 4109};
constexpr TouchRange TOUCH_INDEX  = {4110, 4479};
constexpr TouchRange TOUCH_THUMB  = {4480, 4899};
constexpr TouchRange TOUCH_PALM   = {4900, 5123};

// Finger names
constexpr const char* FINGER_NAMES[NUM_FINGERS] = {
    "Pinky", "Ring", "Middle", "Index", "Thumb_Flex", "Thumb_Abd"
};

}  // namespace inspire_hand_eth

#endif  // INSPIRE_HAND_ETH__REGISTER_MAP_HPP_
