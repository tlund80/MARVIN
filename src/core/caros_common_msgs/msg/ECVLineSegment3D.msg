# Representation for LineSegment3D

# Fields inherited from commonPrimitive3D
# The orientation used for visualisation purposes (normal_vector_plane) is not in here (yet?)
float64 confidence
float64 length
uint32 index
float64[3] position
float64[9] positionCovariance
float64[3] intrinsicDimensionality

# Fields defined in lineSegment3D
int32[2] sourceIndices
float64[2] orientation
float64 orientationConfidence
float64[9] orientationCovariance
float64 phaseAngle
float64 phaseConfidence
float64 phaseVariance
float32[3] colorLeft
float64 colorLeftConfidence
float32[3] colorMiddle
float64 colorMiddleConfidence
float32[3] colorRight
float64 colorRightConfidence


