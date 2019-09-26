# constants for possible gaze targets
int32 NEUTRAL=0 # no person_id need to be given in this case
int32 FACE=1
int32 LEFT_HAND=2
int32 RIGHT_HAND=3
int32 TORSO=4
int32 ROBOT_LEFT_HAND=5
int32 ROBOT_RIGHT_HAND=6
int32 IOTP=7

int32 person_id
int32 gaze_target
float32 min_duration # minimum gaze duration time. node won't accept new targets meanwhile
bool force # forcefully overwrite target in  case of duration block
