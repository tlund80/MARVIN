# Representation for Surfling3D

# Fields inherited from abstractSurfling

#float64 confidence
#float64 length
#uint32 index
uint32 size
uint32[] primitiveIndices
float64[3] position
#float64[3] orientation
float32[3] color
float64 uncertainty
bool boundry
bool doubleBoundaryAtNarrowPassage
float64 boundaryDirectionAngle
float64 boundaryAngle
float64[12] principleComponents
##uint32[] boundaryTypeVector
##int32 BOUNDARY_OWNDED=0
##int32 BOUNDARY_OCCLUSION=1
##int32 BOUNDARY_SHARED=2
##int32 BOUNDARY_TEXTURE_SHADOW=3

# Fields defined in Surfling3D



