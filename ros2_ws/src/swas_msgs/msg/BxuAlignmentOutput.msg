# Alignment error output from any BXU vision node
std_msgs/Header header

float32 lateral_error_m       # Lateral offset from target, in metres (positive = right)
float32 yaw_error_rad         # Yaw misalignment, in radians (positive = clockwise)
float32 distance_m            # Estimated distance to target, in metres (-1 if unknown)

bool markers_detected          # True if at least one valid marker is detected
int32 num_markers_detected     # Number of markers contributing to this estimate
string visibility_class        # e.g. "NONE", "PARTIAL", "FULL" — for FSM decisions
float32 confidence             # 0.0 to 1.0 — quality of current estimate

float32 forward_error_m        # Signed robot-frame forward-axis error (ex_r), in metres.
                                # Only populated by docking vision; 0.0 for front vision.