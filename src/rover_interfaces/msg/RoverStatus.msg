std_msgs/Header header

# Servo angles [front_left, front_right, rear_left, rear_right] (rad)
float32[4] servo_angles

# IMU data from ESP32
float32[3] orientation_euler  # roll, pitch, yaw (radians)
float32[3] linear_acceleration
float32[3] angular_velocity

# System state: 0=idle, 1=navigating, 2=obstacle_avoidance, 3=checkpoint_scan, 4=error
uint8 state

# Last registered checkpoint ID
string last_checkpoint

uint8 STATE_IDLE = 0
uint8 STATE_NAVIGATING = 1
uint8 STATE_OBSTACLE_AVOIDANCE = 2
uint8 STATE_CHECKPOINT_SCAN = 3
uint8 STATE_ERROR = 4
