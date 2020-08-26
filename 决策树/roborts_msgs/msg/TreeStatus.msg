int32    CHASE   = 1
int32    ESCAPE  = 2
int32    SHOOT   = 3
int32    BUFF    = 4
# discribe the status of behavior tree
bool is_running
# 1-追逐，2-逃跑，3-射击，4-找buff
int32 status
# 目标的位置
geometry_msgs/PoseStamped goal