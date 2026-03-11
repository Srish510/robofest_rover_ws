std_msgs/Header header

# Left lane boundary points (in image coordinates)
geometry_msgs/Point[] left_boundary
# Right lane boundary points (in image coordinates)
geometry_msgs/Point[] right_boundary

# Lane center line in rover's base_link frame (meters)
geometry_msgs/Point[] center_line

# Lateral offset from lane center (meters, +ve = right of center)
float64 lateral_offset
# Heading error relative to lane direction (radians)
float64 heading_error

# Detection confidence [0.0 - 1.0]
float64 confidence

# Whether both lanes are detected
bool left_detected
bool right_detected
