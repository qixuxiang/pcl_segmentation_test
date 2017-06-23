uint8 stable
uint8 uptime
time timestamp
float32 vy
float32 fieldAngle
bool lower_board_started
bool lower_board_connected

geometry_msgs/Vector3 curPlat
geometry_msgs/Vector3 deltaData
geometry_msgs/Vector3 robotCtrl
dmotion/ActionCommand action

uint8 STABLE     = 0
uint8 RIGHTDOWN  = 1
uint8 LEFTDOWN   = 2
uint8 FRONTDOWN  = 3
uint8 BACKDOWN   = 4
