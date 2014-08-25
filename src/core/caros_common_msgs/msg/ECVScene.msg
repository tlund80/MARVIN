#Representation for ECV Scene
Header header

StereoCalibration stereoCalibRect
sensor_msgs/Image leftImage
sensor_msgs/Image rightImage

ECVLineSegments3D ls3D

ECVExtendedLineSegments3D els3D

ECVLinks linksLs3D

ECVContours3D contours3D

ECVTexlets3D tex3D

ECVTexlets2D tex2D

ECVLinks linksTex3D

ECVSurflings3D surflings3D

ECVSurfaces3D surfaces3D

ECVSceneRelations texletRelations

# RGB value of White Reference Point
float32[3] whitePoint 


# Things not needed for Milas app. Maybe later.
#ECVLineSegment2D[] ls2DLeft
#ECVLineSegment2D[] ls2DRight
#ECVLinks linksLs2DLeft
#ECVLinks linksLs2DRight
