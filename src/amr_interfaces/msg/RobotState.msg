uint8 MODE_BOOT=0
uint8 MODE_INIT=1
uint8 MODE_MANUAL=2
uint8 MODE_AUTO_READY=3
uint8 MODE_AUTO_RUNNING=4
uint8 MODE_PAUSED=5
uint8 MODE_CHARGING=6
uint8 MODE_FAULT=7
uint8 MODE_ESTOP=8

std_msgs/Header header

uint8 mode
bool fault_active
bool estop_active
bool battery_low
bool battery_critical
bool motor_fault
bool communication_fault
bool docked

string message

