uint8 STATE_ACCEPT_ALL_STATE        = 0
uint8 STATE_DEAD_MAN_STATE          = 1
uint8 STATE_JOY_STATE               = 2
uint8 STATE_AUTONOMUS_STATE         = 3

string DESCRIPTION_ACCEPT_ALL_STATE = "AcceptAllState"
string DESCRIPTION_DEAD_MAN_STATE   = "DeadManState"
string DESCRIPTION_JOY_STATE        = "JoyState"
string DESCRIPTION_AUTONOMUS_STATE  = "AutonomousState"

std_msgs/Header header
uint8 state
string description