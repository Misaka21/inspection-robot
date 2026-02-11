# System-level state snapshot for /inspection/state
std_msgs/Header header

string task_id

uint8 phase
uint8 PHASE_UNSPECIFIED=0
uint8 PHASE_IDLE=1
uint8 PHASE_LOCALIZING=2
uint8 PHASE_PLANNING=3
uint8 PHASE_EXECUTING=4
uint8 PHASE_PAUSED=5
uint8 PHASE_COMPLETED=6
uint8 PHASE_FAILED=7
uint8 PHASE_STOPPED=8

float32 progress_percent
string current_action
string error_message

AgvStatus agv
ArmStatus arm
