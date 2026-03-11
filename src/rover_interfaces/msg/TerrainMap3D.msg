# 3D terrain map produced from SLAM point cloud analysis
std_msgs/Header header

# Map metadata
float64 resolution         # meters per cell (XY plane)
uint32 width               # cells in X
uint32 height              # cells in Y
geometry_msgs/Pose origin  # map origin in world frame

# Flat grids (row-major, width * height) - backward compatible with TerrainMap
uint8[] terrain_classes
float32[] elevation
uint8[] traversability

# 3D terrain cell array with full per-cell data
rover_interfaces/TerrainCell[] cells

# Slope grid (radians, row-major)
float32[] slope

# Roughness grid (meters std dev, row-major)
float32[] roughness

# Confidence grid [0.0-1.0] (row-major)
float32[] confidence

# Statistics
float32 min_elevation
float32 max_elevation
float32 mean_elevation
uint32 total_points_processed

uint8 TERRAIN_UNKNOWN = 0
uint8 TERRAIN_FLAT = 1
uint8 TERRAIN_ROUGH = 2
uint8 TERRAIN_OBSTACLE = 3
uint8 TERRAIN_LANE_BOUNDARY = 4
uint8 TERRAIN_SLOPE = 5
uint8 TERRAIN_STEP = 6
