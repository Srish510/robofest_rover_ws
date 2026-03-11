std_msgs/Header header

# Map metadata
float64 resolution       # meters per cell
uint32 width             # cells
uint32 height            # cells
geometry_msgs/Pose origin  # map origin in world frame

# Terrain classification grid (row-major)
# 0=unknown, 1=flat, 2=rough, 3=obstacle, 4=lane_boundary
uint8[] terrain_classes

# Elevation grid (meters, row-major, NaN for unknown)
float32[] elevation

# Traversability cost grid [0-255] (row-major, 255=impassable)
uint8[] traversability

uint8 TERRAIN_UNKNOWN = 0
uint8 TERRAIN_FLAT = 1
uint8 TERRAIN_ROUGH = 2
uint8 TERRAIN_OBSTACLE = 3
uint8 TERRAIN_LANE_BOUNDARY = 4
