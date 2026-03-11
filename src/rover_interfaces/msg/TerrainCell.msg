# Single 3D terrain cell with full elevation and classification data
float32 x           # cell center X in map frame (meters)
float32 y           # cell center Y in map frame (meters)
float32 elevation   # ground elevation (meters)
float32 slope       # slope angle (radians)
float32 roughness   # surface roughness metric (std dev of height)
uint8 terrain_class # classification (see TerrainMap constants)
uint8 traversability # cost [0-255], 255=impassable
float32 confidence  # mapping confidence [0.0-1.0]
