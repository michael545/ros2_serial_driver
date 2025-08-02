# Message to represent firmware status
uint8 STATUS_OK=0
uint8 STATUS_WARNING=1
uint8 STATUS_ERROR=2

std_msgs/Header header
uint8 status
string message
