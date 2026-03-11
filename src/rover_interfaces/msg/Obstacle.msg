std_msgs/Header header

# Obstacle position relative to rover base_link (meters)
geometry_msgs/Point position
# Obstacle bounding box dimensions (meters)
geometry_msgs/Vector3 size

# Distance to obstacle (meters)
float64 distance
# Bearing angle to obstacle (radians, 0 = straight ahead)
float64 bearing

# Velocity of obstacle (for dynamic obstacle tracking)
geometry_msgs/Vector3 velocity
# Whether the obstacle is moving
bool is_dynamic

# Classification: 0=unknown, 1=rock, 2=person, 3=vehicle, 4=other
uint8 classification
# Detection confidence [0.0 - 1.0]
float64 confidence

uint8 UNKNOWN = 0
uint8 ROCK = 1
uint8 PERSON = 2
uint8 VEHICLE = 3
uint8 OTHER = 4
