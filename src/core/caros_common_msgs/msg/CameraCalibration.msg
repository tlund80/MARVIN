uint32 height
uint32 width

# Distortion parameters
float64[4] D

# Intrinsic parameters
float64[9] K # 3x3 row-major matrix

# Extrinsic parameters
float64[9] R # 3x3 row-major matrix
float64[3] t
