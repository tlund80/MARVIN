# Representation for Texlet3D

# Fields inherited from commonPrimitive3D
# For the texlet the orientation is the same as is used for visualisation purpose (normal_vector_plane)
float64 confidence
float64 length
uint32 index
float64[3] position
float64[9] positionCovariance
float64[3] orientation
float64[3] intrinsicDimensionality

# Fields defined in texlet3D
int32 sourceSegment
float64[9] orientationCovariance
float32[3] color

