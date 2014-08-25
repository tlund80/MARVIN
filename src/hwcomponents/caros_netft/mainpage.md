
\mainpage
\htmlinclude manifest.html

[TOC]

This component enables the stereo calibration of kinect cameras using the standard 
stereo calibration component from ros (camera__calibration).

Since the kinect cannot stream both at the same time this component will buffer one stream
and forward the other stream. It will also allow one to switch between which stream is forwarded 
and which stream is buffered. Finally, it allows one to grab a single image from the buffered 
stream which will be synchronized with the forwarded stream. 

This firstly enables one to use the standard ros stereo calibration gui but also allow
one to more clearly control which images to use in the stereo calibration. 


# Generic kinect calibration process # 

First use openni to start the kinect camera drivers 

	roslaunch openni__launch openni.launch

This should initialize the camera and start publishing images on /camera/rgb and /camera/ir.

Next start the kinect__calibration component.

	rosrun kinect__calibration kinect__stereo__camera

This component will start publishing two new image streams /camera/right and /camera/left, where
right is the rgb camera and ir is the left camera. The terminal in which this component
runs will be used for controlling the image streams.

Finally start the standard ros stereo calibration component and remember to edit the --size 
and --square attributes to fit the physical attributes of your calibration plate. See 
[ROS stereo calibration](http://www.ros.org/wiki/camera_calibration/Tutorials/StereoCalibration) for 
more details.     

	rosrun camera__calibration cameracalibrator.py --size 4x6 --square 0.0518 
		   right:=/camera/right/image left:=/camera/left/image right__camera:=/camera/right left__camera:=/camera/left 

Now the stereo camera view should startup and you should see something like:

![RGB view of stereo calibration](images/first-view-stereo.png)

Now go to the terminal of kinect__calibration and press s. This should switch the streaming from 
RGB to IR as in the following image:

![IR view of stereo calibration](images/second-view-stereo.png)

Now if the calibration board is detected in both image streams press 'g'. This will publish the buffered image 
for roughly 1 second which is enough for the stereo calibration to add a sample. If a sample was added an output 
similar to the following should be displayed in the camera__calibration terminal 
	
	...
	*** Added sample 1, p_x = 0.383, p_y = 0.549, p_size = 0.165, skew = 0.008
	...
 

# Marvin kinect calibration process #

