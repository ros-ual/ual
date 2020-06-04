# UAL state constants
uint8 UNINITIALIZED   = 0
uint8 LANDED_DISARMED = 1
uint8 LANDED_ARMED    = 2
uint8 TAKING_OFF      = 3
uint8 FLYING_AUTO     = 4
uint8 FLYING_MANUAL   = 5
uint8 LANDING         = 6

std_msgs/Header header
uint8 state         # The UAL state
string flight_mode  # Autopilot custom flight mode if any
sensor_msgs/BatteryState battery
