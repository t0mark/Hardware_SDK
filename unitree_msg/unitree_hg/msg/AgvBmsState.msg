# AGV Battery Management System state for H1/G1 humanoid robot
# Source: unitree_sdk2/include/unitree/idl/hg/AgvBmsState_.hpp

string software_version     # BMS software version
uint8 battery_percentage    # Battery level (0-100)
int32 current               # Current [mA]
int16[3] temperature        # Temperature readings from 3 sensors [°C]
string docking_status       # Docking station status
bool is_charging            # Charging flag
bool is_dc_connected        # DC power connected flag
