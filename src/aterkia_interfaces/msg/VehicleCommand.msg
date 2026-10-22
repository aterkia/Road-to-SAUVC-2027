# Onboard guidance input, not velocity or individual motor power.
# FLU base_link; finite axes in [-1, 1]. Source stamps must advance.
std_msgs/Header header
float64 surge_input
float64 yaw_input
float64 heave_input
