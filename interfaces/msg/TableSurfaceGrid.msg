uint8 UNKNOWN=0
uint8 FREE=1
uint8 BLOCKED=2

std_msgs/Header header
float64 resolution_m
float64 x_min_m
float64 y_min_m
uint32 width
uint32 height
# Row-major cells: index = y_index * width + x_index.
uint8[] cells
