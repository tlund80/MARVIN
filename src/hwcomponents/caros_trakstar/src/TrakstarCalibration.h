/*
 * TrakstarCalibration.hpp
 *
 *  Created on: Jan 20, 2012
 *      Author: liljekrans
 */

#ifndef TRAKSTARCALIBRATION_HPP_
#define TRAKSTARCALIBRATION_HPP_

#include <rw/math.hpp>
#include <string>
#include "ros/ros.h"
#include <boost/multi_array.hpp>


class TrakstarCalibration {
private:

	// ROS
	bool								_rosInitialized;
	ros::NodeHandle 					_nodeHandle;

	typedef boost::multi_array<rw::math::Vector3D<double>, 3> errorLutType;
	errorLutType 						_errorLut;


	rw::math::Vector3D<int>				_lutDimensions;		// Dimensions of the matrix
	rw::math::Vector3D<double>			_lutOffset;
	int									_lutStepSize;

	void saveLUT();
	bool loadLUT();
	bool 								_lutLoaded;


public:
	TrakstarCalibration();
	virtual ~TrakstarCalibration();

	// Calculate mean and std.dev from a set of transforms (this is a basic math operation put in this class)
	void CalculateAndSaveTransforms(std::vector<std::vector<rw::math::Transform3D<double> > > transforms);

	// Get updated/calibrated position using _Cx/_Cy/_Cz
	bool getCalibratedPosition(double &x, double &y, double &z);

	// Perform calibration
	void InternalPositionCalibration(const std::vector<rw::math::Transform3D<double> > &tBaseSensorMeasuredRecords,const std::vector<rw::math::Transform3D<double> > &tBaseSensorTrueRecords, double stepSize);

	//
	bool reloadCalibration();


	void calcMeanTransformation(std::vector<rw::math::Transform3D<double> > transforms, rw::math::Transform3D<double> &avgTransform, rw::math::Vector3D<double> &stdDev);
	void saveBaseTransmitterTransformation(rw::math::Transform3D<double> transform);
	bool loadBaseTransmitterTransform(rw::math::Transform3D<double> &transform);

};

#endif /* TRAKSTARCALIBRATION_HPP_ */
