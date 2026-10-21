# Image-space observation from the front YOLO vision pipeline.
# All pixel values are in the rectified front-camera image frame.
# This message intentionally contains no metric pose estimate.

std_msgs/Header header

bool valid
string state

uint32 image_width
uint32 image_height
float32 principal_point_x_px

# Smoothed image-space target centre and its horizontal error.
float32 center_x_px
float32 center_y_px
float32 center_error_x_px
float32 center_error_x_normalized

# Angle of the line between the two primary wheel centres.
# This is an image feature, not a metric vehicle yaw estimate.
bool wheel_line_valid
float32 wheel_line_angle_rad

uint8 real_wheels
uint8 held_wheels
bool confirmed_pair
bool wheel_only_mode

# Primary wheel pair. The three arrays always have equal length.
float32[] wheel_box_xyxy_px
bool[] wheel_held
string[] wheel_side

float32 detector_confidence