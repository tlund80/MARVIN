/*
 * TrakstarCalibration.cpp
 *
 *  Created on: Jan 20, 2012
 *      Author: liljekrans
 */

#include "TrakstarCalibration.h"
#include <iostream>
#include <vector>

#include <sstream>
#include <fstream>
#include <algorithm>
#include <map>

// ROS
#include "ros/ros.h"
#include <XmlRpcValue.h>

// Boost
#include <boost/filesystem.hpp>
#include "boost/multi_array.hpp"

// RW
#include <rw/math.hpp>
#include <rw/common/PropertyMap.hpp>
#include <rw/loaders/xml/XMLPropertyLoader.hpp>
#include <rw/loaders/xml/XMLPropertySaver.hpp>
#include <rw/common.hpp>

using namespace std;
using namespace rw::math;

namespace {
bool initializeRos() {
	static bool hasRun = false;
	if (!hasRun) {
		hasRun = true;
		char** argv = NULL;
		int argc = 0;
		ros::init(argc, argv, "TrakstarCalibPlugin");
	}
	return true;
}

template<typename BidirectionalIterator>
BidirectionalIterator getClosest(BidirectionalIterator first,
		BidirectionalIterator last, const double & value) {
	BidirectionalIterator before = std::lower_bound(first, last, value);

	if (before == first)
		return first;
	if (before == last)
		return --last; // iterator must be bidirectional

	BidirectionalIterator after = before;
	--before;
	cout << "after: " << *after << endl;

	return (*after - value) < (value - *before) ? after : before;
}

template<typename BidirectionalIterator, typename T>
std::size_t getClosestIndex(BidirectionalIterator first,
		BidirectionalIterator last, const T & value) {
	return std::distance(first, getClosest(first, last, value));
}

int getPositionOfLevel(vector<double> values, double level) {
	return getClosestIndex(values.begin(), values.end(), level);
}
}

TrakstarCalibration::TrakstarCalibration() :
//		_rosInitialized(initializeRos()),
//		_nodeHandle(ros::NodeHandle()),
		_errorLut(boost::extents[1][1][1]) {

	_lutOffset = Vector3D<>(0, 0, 0);
	_lutStepSize = 50;

	if (loadLUT()) {
		_lutLoaded = true;
	} else {
		_lutLoaded = false;
	}
}

TrakstarCalibration::~TrakstarCalibration() {
	//std::cout << "TrakstarCalibration Destructor" << std::endl;
}

/*
 * InternalPositionCalibration
 *
 * Calculate error vectors and prepare LUT for "getCalibratedPosition()"
 */
void TrakstarCalibration::InternalPositionCalibration(
		const vector<Transform3D<double> > &tMeasured,
		const vector<Transform3D<double> > &tTrue, double stepSize) {
	_lutStepSize = 2 * stepSize * 1000;
	// Find largest index values to determine size of _errorLut.
	double maxX = -10000;
	double maxY = -10000;
	double maxZ = -10000;
	double minX = 10000;
	double minY = 10000;
	double minZ = 10000;
	vector<Transform3D<double> >::const_iterator itMeasured;
	vector<Transform3D<double> >::const_iterator itTrue;

	for (itMeasured = tMeasured.begin(); itMeasured != tMeasured.end();
			itMeasured++) {
		Vector3D<double> position = itMeasured->P();
		if (maxX < position[0])
			maxX = position[0];
		if (maxY < position[1])
			maxY = position[1];
		if (maxZ < position[2])
			maxZ = position[2];
		if (minX > position[0])
			minX = position[0];
		if (minY > position[1])
			minY = position[1];
		if (minZ > position[2])
			minZ = position[2];
	}
	cout << "max1: " << maxX << ", " << maxY << ", " << maxZ << endl;
	cout << "min1: " << minX << ", " << minY << ", " << minZ << endl;

	// Save offset
	_lutOffset = Vector3D<>(minX, minY, minZ);

	// Calc dimensions of LUT
	if (maxX < 0) {
		maxX = maxX - minX;
		minX = 0;
	}
	if (maxY < 0) {
		maxY = maxY - minY;
		minY = 0;
	}
	if (maxZ < 0) {
		maxZ = maxZ - minZ;
		minZ = 0;
	}
	cout << "max2: " << maxX << ", " << maxY << ", " << maxZ << endl;
	cout << "min2: " << minX << ", " << minY << ", " << minZ << endl;

	double maxIndexX = floor((maxX - minX) / _lutStepSize) + 1;
	double maxIndexY = floor((maxY - minY) / _lutStepSize) + 1;
	double maxIndexZ = floor((maxZ - minZ) / _lutStepSize) + 1;

	cout << "Dimensions, xyz: " << maxIndexX << ", " << maxIndexY << ", "
			<< maxIndexZ << endl;

	// Resize _errorLut
	_errorLut.resize(boost::extents[maxIndexX][maxIndexY][maxIndexZ]);

	// Put error-values into _errorLut where tMeasured is used as index
	RW_ASSERT(tMeasured.size() == tTrue.size());

	// tmp array with list of error-values
	boost::multi_array<vector<Vector3D<double> >, 3> tmpErrorList;

	tmpErrorList.resize(boost::extents[maxIndexX][maxIndexY][maxIndexZ]);

	// LUT with a vector of errors at each node. Must be averaged afterwards and saved in _errorLut;
	itTrue = tTrue.begin();
	itMeasured = tMeasured.begin();
	while (itMeasured != tMeasured.end()) {
		Vector3D<double> position = itMeasured->P() - _lutOffset;
		double indexX = floor(position[0] / _lutStepSize);
		double indexY = floor(position[1] / _lutStepSize);
		double indexZ = floor(position[2] / _lutStepSize);
		//cout << __LINE__ << endl;
		tmpErrorList[indexX][indexY][indexZ].push_back(
				itMeasured->P() - itTrue->P());
		itMeasured++;
		itTrue++;
	}

	// Calc average of errors and put result into _errorLut;
	for (int i = 0; i < maxIndexX; i++) {
		for (int j = 0; j < maxIndexY; j++) {
			for (int k = 0; k < maxIndexZ; k++) {
				Vector3D<double> sum(0, 0, 0);
				if (tmpErrorList[i][j][k].size() > 0) {
					for (vector<Vector3D<double> >::iterator it =
							tmpErrorList[i][j][k].begin();
							it != tmpErrorList[i][j][k].end(); it++) {
						sum += *it;
					}
					_errorLut[i][j][k] = sum / tmpErrorList[i][j][k].size();
				} else {
					_errorLut[i][j][k] = sum;
				}
				cout << "sum error[" << i << "," << j << "," << k << "]: "
						<< _errorLut[i][j][k] << endl;
			}
		}
	}
	cout << "Finished saving error vectors" << endl;

	// Save to ROS Parameter Server
	saveLUT();

	// Save errors to errors.txt for printing with gnuplot //
	{
		ofstream out;
		out.open("/home/liljekrans/work/errors.txt", ios_base::trunc);
		if (!out) {
			cerr << "Could not open file in TrakstarCalibration.cpp, line"
					<< __LINE__ << endl;
			return;
		}

		int i = 0;
		out << "splot ";
		for (itMeasured = tMeasured.begin(); itMeasured != tMeasured.end();
				itMeasured++) {
			out << "\"errors.txt\" using " << (i + 1) << ":" << (i + 2) << ":"
					<< (i + 3) << " with lines, ";
			i += 3;
		}
		out << endl;
		for (itMeasured = tMeasured.begin(); itMeasured != tMeasured.end();
				itMeasured++) {
			out << itMeasured->P()[0] << " " << itMeasured->P()[1] << " "
					<< itMeasured->P()[2] << " ";
		}
		out << endl;
		for (itTrue = tTrue.begin(); itTrue != tTrue.end(); itTrue++) {
			out << itTrue->P()[0] << " " << itTrue->P()[1] << " "
					<< itTrue->P()[2] << " ";
		}

		// write xyz for all true values

		out.close();
	}
}

bool TrakstarCalibration::getCalibratedPosition(double &xRaw, double &yRaw,
		double &zRaw) {

	static bool firstRun = true;
	if (!_lutLoaded) {
		if (firstRun) {
			cerr
					<< "No Lookup Table was loaded for Trakstar Internal Calibration. Throughpassing values."
					<< endl;
			firstRun = false;
		}
		cerr << "LUT not loaded" << std::endl;
		return false;
	}
	//cerr << "LUT IS loaded" << std::endl;
	if (firstRun) {
		firstRun = false;
		cout << "Lookup table dimensions: ";
		cout << _errorLut.shape()[0] << ", ";
		cout << _errorLut.shape()[1] << ", ";
		cout << _errorLut.shape()[2] << endl;

		cout << "dMax: " << _lutStepSize / 2 << endl;
	}

	// offset point so that its range is 0 - MAX in both x, y, z.
	Vector3D<double> pos(xRaw - _lutOffset[0], yRaw - _lutOffset[1],
			zRaw - _lutOffset[2]);
	//cerr << "pos " << pos << endl;

	// calculate indexes
	/*int iX = std::abs((int)floor(pos[0] / _lutStepSize));
	 int iY =  std::abs((int)floor(pos[1] / _lutStepSize));
	 int iZ =  std::abs((int)floor(pos[2] / _lutStepSize));*/
	unsigned int iX = (int) floor(pos[0] / _lutStepSize);
	unsigned int iY = (int) floor(pos[1] / _lutStepSize);
	unsigned int iZ = (int) floor(pos[2] / _lutStepSize);
	Vector3D<double> correction;

	// The index is within LUT bounds
	//cerr << iX << " < " << (_errorLut.shape()[0] - 1) << endl;
	//cerr << iY << " < " << (_errorLut.shape()[0] - 1) << endl;
	//cerr << iZ << " < " << (_errorLut.shape()[0] - 1) << endl;
	if (iX < (_errorLut.shape()[0] - 1) && iY < (_errorLut.shape()[0] - 1)
			&& iZ < (_errorLut.shape()[0] - 1)) {

		double w[8]; // weight that each node's error should have (normalised)
		double r[8]; // weightfunction (unscaled).
		double d[8]; // distance from the tracked point to the surrounding nodes
		double rsum = 0;
		double dmax = 43; // half of the longest diagonal
		Vector3D<double> nIndex[8]; // surrounding node indexes
		nIndex[0] = Vector3D<double>(iX, iY, iZ);
		nIndex[1] = Vector3D<double>(iX + 1, iY, iZ);
		nIndex[2] = Vector3D<double>(iX, iY + 1, iZ);
		nIndex[3] = Vector3D<double>(iX + 1, iY + 1, iZ);
		nIndex[4] = Vector3D<double>(iX, iY, iZ + 1);
		nIndex[5] = Vector3D<double>(iX + 1, iY, iZ + 1);
		nIndex[6] = Vector3D<double>(iX, iY + 1, iZ + 1);
		nIndex[7] = Vector3D<double>(iX + 1, iY + 1, iZ + 1);

		//cout << "start:" << endl;
		//cout << "pos: " << pos << endl;
		// Calculate unscaled weights
		for (int n = 0; n < 8; n++) {
			Vector3D<double> f = nIndex[n] * _lutStepSize; // node location
			//cout << "n=" << n << ": ";
			//cout << f << ", ";
			// Distance
			Vector3D<> tmp = f - pos;
			d[n] = tmp.norm2();
			//cout << d[n] << endl;
		}

		//cout << endl;
		//cout << "r[n]: ";
		for (int n = 0; n < 8; n++) {
			// Weight function
			/*if (d[n] > dmax) {
			 r[n] = 0;
			 } else {
			 r[n] = exp(-d[n]/10);
			 }*/

			r[n] = exp(-d[n] / 10); // Exponential	//TODO: Produce plot that shows errors when this is used (as VIS3 plots)

			// Linear (0 at distant node, 1 at local node.. thingy).
			/*if (d[n] > dmax) {
			 r[n] = 0;
			 } else {
			 r[n] = 1 - (d[n] / _lutStepSize);
			 }*/

			/*
			 // Linear					// Changes abruptly when changing grids
			 bool first = true;
			 for (int pr = 0; pr < 8; pr++) {
			 if (pr != n && d[pr] != 0) {
			 if (first) {
			 r[n] = d[pr];
			 first = false;
			 } else
			 r[n] *= d[pr];
			 }
			 }*/
			cout << r[n] << ", ";
			rsum += r[n];
		}
		//cout << endl;

		/*cout << "d[n]: ";
		 for (int n = 0; n < 8; n++)
		 cout << d[n] << ", ";
		 cout << endl;
		 cout << "r[n]: ";
		 for (int n = 0; n < 8; n++)
		 cout << r[n] << ", ";
		 cout << endl;
		 cout << "rsum: " << rsum << endl;*/
		//cout << "w[n] & correction: " << endl;
		// Normalisation and final correction
		for (int n = 0; n < 8; n++) {
			w[n] = r[n] / rsum;
			//cout << w[n] << ", ";
			//cout << "[ " << (nIndex[n])[0] << ", " << (nIndex[n])[1] << ", " << (nIndex[n])[2] << "] ";
			//cout << _errorLut[(nIndex[n])[0]][(nIndex[n])[1]][(nIndex[n])[2]] << endl;
			correction +=
					_errorLut[(nIndex[n])[0]][(nIndex[n])[1]][(nIndex[n])[2]]
							* w[n];
		}

	} else {

		// Signal that no value was found (out of bounding box)
		//cerr << "Signal that no value was found (out of bounding box)" << endl;
		return false;
	}

	xRaw += correction[0];
	yRaw += correction[1];
	zRaw += correction[2];
	//cerr << "Value changed by: " << correction[0] << ", " << correction[1] << ", " << correction[2] << endl;

	return true;

}

void TrakstarCalibration::calcMeanTransformation(
		vector<Transform3D<double> > transforms,
		Transform3D<double> &avgTransform, Vector3D<double> &stdDev) {

	RW_ASSERT(transforms.size() > 0);

	// position and rotation variables
	Vector3D<double> positionSum;
	Quaternion<double> rotationSum(0, 0, 0, 0);

	// statistics
	vector<Vector3D<double> > tmpDiff;

	// Sum
	for (unsigned int i = 0; i < transforms.size(); i++) {
		positionSum += transforms[i].P();
		Quaternion<double> tmpRotation(transforms[i].R());
		rotationSum += tmpRotation;
	}

	// Calc average position and rotation
	Vector3D<double> avgPosition = positionSum / transforms.size();
	Quaternion<double> avgRotation = rotationSum
			* (1. / rotationSum.getLength()); // FIXLB
	avgTransform = Transform3D<double>(avgPosition, avgRotation.toRotation3D());

	// Calculate position standard deviation
	Vector3D<double> tmp;
	for (unsigned int i = 0; i < transforms.size(); i++) {
		Vector3D<double> tmpV3d = transforms[i].P() - avgPosition;
		tmp[0] += tmpV3d[0] * tmpV3d[0];
		tmp[1] += tmpV3d[1] * tmpV3d[1];
		tmp[2] += tmpV3d[2] * tmpV3d[2];
	}
	tmp /= transforms.size();
	tmp[0] = sqrt(tmp[0]);
	tmp[1] = sqrt(tmp[1]);
	tmp[2] = sqrt(tmp[2]);

	stdDev = tmp;
}

/*
 * CalculateMeanVarianceFromTransforms
 *
 * Given a set of transforms (each vector is a set of vectors of transformations)
 * meaning that transforms[0][0] is from the same pose as transforms[1][0].
 */
void TrakstarCalibration::CalculateAndSaveTransforms(
		vector<vector<Transform3D<double> > > transforms) {

	RW_ASSERT(transforms.size() > 0);
	RW_ASSERT(transforms[0].size() > 0);

	vector<Vector3D<double> > sum;
	sum.assign(transforms[0].size(), Vector3D<double>(0, 0, 0));

	int rounds = transforms.size();
	int nodes = transforms[0].size();

	ofstream out;
	out.open("/home/liljekrans/work/distortiontest.txt", ios_base::trunc);
	if (!out) {
		cerr << "Could not open file in TrakstarCalibration.cpp, line"
				<< __LINE__ << endl;
		return;
	}

	out << "rounds: " << rounds << ", nodes: " << nodes << endl;

	out.precision(15);
	for (int round = 0; round < rounds; round++) {
		for (int node = 0; node < nodes; node++) {
			rw::math::Quaternion<double> q(transforms[round][node].R());
			out << transforms[round][node].P()[0] << ", "
					<< transforms[round][node].P()[1] << ", "
					<< transforms[round][node].P()[2] << ", ";
			out << q(0) << ", " << q(1) << ", " << q(2) << ", " << q(3) << ", ";
			cout << transforms[round][node].P() << endl;
		}
		out << endl;
	}

	out.close();

	cout << "Data saved." << endl;
}

bool TrakstarCalibration::loadBaseTransmitterTransform(
		rw::math::Transform3D<double> &transform) {

	XmlRpc::XmlRpcValue listPos, listRot;

	if (!ros::param::get(
			"trakstar/calibration/external/t_base_transmitter/position",
			listPos)) {
		RW_THROW(
				"Could not load calibration parameters. Is the ROS PARAM server loaded with values");
	}
	if (!ros::param::get(
			"trakstar/calibration/external/t_base_transmitter/rotation",
			listRot)) {
		RW_THROW(
				"Could not load calibration parameters. Is the ROS PARAM server loaded with values");
	}

	ROS_ASSERT(listPos.getType() == XmlRpc::XmlRpcValue::TypeArray);
	ROS_ASSERT(listPos.size() == 3);
	ROS_ASSERT(listRot.getType() == XmlRpc::XmlRpcValue::TypeArray);
	ROS_ASSERT(listRot.size() == 9);

	Transform3D<double> tmpTransform;
	// Position
	for (int i = 0; i < 3; i++) {
		ROS_ASSERT(listPos[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
		tmpTransform.P()[i] = listPos[i];
	}
	// Rotation
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			ROS_ASSERT(
					listRot[3*i+j].getType() == XmlRpc::XmlRpcValue::TypeDouble);
			tmpTransform.R()(j, i) = listRot[3 * i + j];
		}
	}
	transform = tmpTransform;
	return true;
}

void TrakstarCalibration::saveBaseTransmitterTransformation(
		Transform3D<double> transform) {

	XmlRpc::XmlRpcValue listPos, listRot;
	listPos.setSize(3);
	listRot.setSize(9);

	// Position
	for (int i = 0; i < 3; i++) {
		listPos[i] = transform.P()[i];
	}

	// Rotation
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			listRot[3 * i + j] = transform.R()(j, i);
		}
	}

	// Save coefficients
	ros::param::set("trakstar/calibration/external/t_base_transmitter/position",
			listPos);
	ros::param::set("trakstar/calibration/external/t_base_transmitter/rotation",
			listRot);

}

bool TrakstarCalibration::reloadCalibration() {
	return loadLUT();
}

/*
 * loadLUT
 *
 * Loads the LUT into _errorLUT from ROS Parameter server
 */
bool TrakstarCalibration::loadLUT() {

	XmlRpc::XmlRpcValue LUT_Size, LUT_Values, LUT_Offset, LUT_StepSize;

	if (!ros::param::has("trakstarNodeletManager/calibration/internal/LUT_size")
			|| !ros::param::has(
					"trakstarNodeletManager/calibration/internal/LUT_values")
			|| !ros::param::has(
					"trakstarNodeletManager/calibration/internal/LUT_offset")
			|| !ros::param::has(
					"trakstarNodeletManager/calibration/internal/LUT_stepsize")) {
		cerr
				<< "TrakstarCalibration::loadLUT(): ros::param, parameters not available."
				<< endl;
		return false;
	}
	// LUT Sizes
	ros::param::get("trakstarNodeletManager/calibration/internal/LUT_size",
			LUT_Size);
	ROS_ASSERT(LUT_Size[0].getType() == XmlRpc::XmlRpcValue::TypeInt);
	RW_ASSERT(LUT_Size.size() == 3);
	for (int i = 0; i < 3; i++) {
		_lutDimensions[i] = static_cast<int>(LUT_Size[i]);
	}

	// LUT values
	ros::param::get("trakstarNodeletManager/calibration/internal/LUT_values",
			LUT_Values);
	cout << "Lut values.size: " << LUT_Values.size() << endl;
	cout << "Lut dimensions: " << _lutDimensions[0] << ", " << _lutDimensions[1]
			<< ", " << _lutDimensions[2] << endl;

	RW_ASSERT(
			LUT_Values.size()
					== (_lutDimensions[0] * _lutDimensions[1]
							* _lutDimensions[2] * 3));
	_errorLut.resize(
			boost::extents[_lutDimensions[0]][_lutDimensions[1]][_lutDimensions[2]]);
	int i = 0, j = 0, k = 0;
	Vector3D<double> error;
	for (int d = 0; d < LUT_Values.size(); d += 3) {
		ROS_ASSERT(LUT_Values[d].getType() == XmlRpc::XmlRpcValue::TypeDouble);
		error = Vector3D<>(static_cast<double>(LUT_Values[d]),
				static_cast<double>(LUT_Values[d + 1]),
				static_cast<double>(LUT_Values[d + 2]));
		_errorLut[i][j][k] = error;
		k++;
		if (k == _lutDimensions[2]) {
			k = 0;
			j++;
		}
		if (j == _lutDimensions[1]) {
			j = 0;
			i++;
		}
		if (j == _lutDimensions[0]) {
			i = 0;
		}
	}

	// LUT Offset
	ros::param::get("trakstarNodeletManager/calibration/internal/LUT_offset",
			LUT_Offset);
	RW_ASSERT(LUT_Offset.size() == 3);
	ROS_ASSERT(LUT_Offset[0].getType() == XmlRpc::XmlRpcValue::TypeDouble);
	for (int i = 0; i < 3; i++) {
		_lutOffset[i] = static_cast<double>(LUT_Offset[i]);
	}

	// LUT StepSize
	ros::param::get("trakstarNodeletManager/calibration/internal/LUT_stepsize",
			LUT_StepSize);
	ROS_ASSERT(LUT_StepSize.getType() == XmlRpc::XmlRpcValue::TypeInt);
	_lutStepSize = static_cast<int>(LUT_StepSize);

	return true;
}

/*
 * saveLUT
 *
 * Saves the Error values found in "InternalCalibration"
 *
 */
void TrakstarCalibration::saveLUT() {

	XmlRpc::XmlRpcValue LUT_Size;
	LUT_Size.setSize(3);
	for (int i = 0; i < 3; i++)
		LUT_Size[i] = static_cast<int>(_errorLut.shape()[i]);
	ros::param::set("trakstar/calibration/internal/LUT_size", LUT_Size);

	XmlRpc::XmlRpcValue LUT_Values;
	int iMax = _errorLut.shape()[0];
	int jMax = _errorLut.shape()[1];
	int kMax = _errorLut.shape()[2];
	LUT_Values.setSize(iMax * jMax * kMax * 3);
	for (int i = 0; i < iMax; i++) {
		for (int j = 0; j < jMax; j++) {
			for (int k = 0; k < kMax; k++) {
				for (int xyz = 0; xyz < 3; xyz++) {
					LUT_Values[(k + j * kMax + i * kMax * jMax) * 3 + xyz] =
							(_errorLut[i][j][k])[xyz];
				}
			}
		}
	}

	// Save LUT
	ros::param::set("trakstar/calibration/internal/LUT_values", LUT_Values);

	// Save _lutOffset
	XmlRpc::XmlRpcValue LUT_OffsetValues;
	LUT_OffsetValues.setSize(3);
	for (int i = 0; i < 3; i++)
		LUT_OffsetValues[i] = _lutOffset[i];
	ros::param::set("trakstar/calibration/internal/LUT_offset",
			LUT_OffsetValues);

	// Save _lutStepSize
	XmlRpc::XmlRpcValue LUT_StepSize;
	LUT_StepSize = _lutStepSize;
	ros::param::set("trakstar/calibration/internal/LUT_stepsize", LUT_StepSize);

}

/*
 * preparePermutations
 *
 * Used for Polynomial fitting. Deprecated
 * Calculates permutations according to the order used
 * (e.g. 10 permutations for 2nd order polynomial, 35 permutations for 4th order etc)
 */
#if 0
bool TrakstarCalibration::preparePermutations(unsigned int polynomialOrder) {
	_p.clear();
	vector<int> tmp;
	if (!polynomialOrder) {
		polynomialOrder = _polynomialOrder;
	}
	if (!polynomialOrder) {
		return false;
	}

	tmp.assign(3, 0);
	//cout << "permutations: (order)" << polynomialOrder << endl;
	for (unsigned int x = 0; x <= polynomialOrder; x++) {
		for (unsigned int y = 0; y <= polynomialOrder; y++) {
			for (unsigned int z = 0; z <= polynomialOrder; z++) {
				if (x+y+z <= polynomialOrder) {
					tmp[0] = x;
					tmp[1] = y;
					tmp[2] = z;
					_p.push_back(tmp);
					//cout << tmp[0] << ", " << tmp[1] << ", " << tmp[2] << endl;
				}
			}
		}
	}
	//cout << "done" << endl;
	return true;
}
#endif

/*
 * InternalPositionCalibration
 *
 * Old version of internal calibration using polynomial fit.
 * Deprecated.
 */
#if 0
void TrakstarCalibration::InternalPositionCalibration(const vector<Transform3D<double> > &tMeasured,const vector<Transform3D<double> > &tTrue)
{
	if (_polynomialOrderForNewCalibration > 10 || _polynomialOrderForNewCalibration < 1) {
		_polynomialOrderForNewCalibration = 3;
	}
	int polynomialTerms = (_polynomialOrderForNewCalibration+1)*(_polynomialOrderForNewCalibration+2)*(_polynomialOrderForNewCalibration+3)/6;
//	cout << "polyTerms: " << polynomialTerms << endl;
//	cout << "polyorder: " << _polynomialOrderForNewCalibration << endl;

	// Prepare permutations with (possibly) a new _polynomialOrderForNewCalibration
	preparePermutations(_polynomialOrderForNewCalibration);

	// Fake values
	/*vector<Transform3D<double> > tMeasured;
	 vector<Transform3D<double> > tTrue;
	 tMeasured.push_back(Transform3D<double>(Vector3D<double>(1.1, 2.3, 3.3)));
	 tMeasured.push_back(Transform3D<double>(Vector3D<double>(2.2, 4.6, 4.4)));
	 tMeasured.push_back(Transform3D<double>(Vector3D<double>(3.3, 6.9, 5.6)));

	 tTrue.push_back(Transform3D<double>(Vector3D<double>(1.2, 2.3, 3.5)));
	 tTrue.push_back(Transform3D<double>(Vector3D<double>(2.3, 4.4, 4.6)));
	 tTrue.push_back(Transform3D<double>(Vector3D<double>(3.5, 6.3, 5.9)));
	 */
	// We started a new calibration.
	_coefficientsLoaded = false;

	// Solve equations
	vector<vector<double> > a;// n x m matrix (contains x[i]*p[j][0] * y[i]*p[j][1] * z[i]*p[j][2])
	vector<double> tmp;
	tmp.assign(polynomialTerms,0);
	a.assign(tMeasured.size(), tmp);
	LinearAlgebra::Matrix<double>::type A;// n x n matrix (A in Ax=B)
	LinearAlgebra::Vector<double>::type Bx;// n x 1 matrix (B in Ax=B)
	LinearAlgebra::Vector<double>::type By;// n x 1 matrix (B in Ax=B)
	LinearAlgebra::Vector<double>::type Bz;// n x 1 matrix (B in Ax=B)
	A.resize(polynomialTerms,polynomialTerms,false);
	Bx.resize(polynomialTerms,false);
	By.resize(polynomialTerms,false);
	Bz.resize(polynomialTerms,false);
	_Cx.clear();
	_Cy.clear();
	_Cz.clear();
	_Cx.resize(polynomialTerms,false);
	_Cy.resize(polynomialTerms,false);
	_Cz.resize(polynomialTerms,false);

	cout << "Solving with:" << endl;
	cout << "PolyTerms:" << polynomialTerms << endl;
	cout << "Measured(sensor) values: " << tMeasured.size() << endl;
	cout << "Measured(robot) values: " << tTrue.size() << endl;
	cout << "Permutations _p.size(): " << _p.size() << endl;

	// Rows in the equation system
	for (int k = 0; k < polynomialTerms; k++) {
		// Coefficients
		for (int j = 0; j < polynomialTerms; j++) {
			// Measurements
			double aTmpSum = 0;
			for (unsigned int i = 0; i < tMeasured.size(); i++) {
				if (a[i][j] == 0) {
					a[i][j] = pow(tMeasured[i].P()[0], _p[j][0]) * pow(tMeasured[i].P()[1], _p[j][1]) * pow(tMeasured[i].P()[2], _p[j][2]);

				}
				aTmpSum += a[i][j] * a[i][k];
			}
			A(k, j) = aTmpSum;

		}
		double bxTmpSum = 0;
		double byTmpSum = 0;
		double bzTmpSum = 0;
		cout << "errors in x[mm]: " << endl;
		for (unsigned int i = 0; i < tTrue.size(); i++) {

			cout << "#" << i << ": " << tTrue[i].P()[0] - tMeasured[i].P()[0] << endl;
			bxTmpSum += (tTrue[i].P()[0] - tMeasured[i].P()[0]) * a[i][k];
			byTmpSum += (tTrue[i].P()[1] - tMeasured[i].P()[1]) * a[i][k];
			bzTmpSum += (tTrue[i].P()[2] - tMeasured[i].P()[2]) * a[i][k];
		}
		Bx(k) = bxTmpSum;
		By(k) = byTmpSum;
		Bz(k) = bzTmpSum;
	}

	axpy_prod(rw::math::LinearAlgebra::pseudoInverse(A), Bx, _Cx, true);
	axpy_prod(rw::math::LinearAlgebra::pseudoInverse(A), By, _Cy, true);
	axpy_prod(rw::math::LinearAlgebra::pseudoInverse(A), Bz, _Cz, true);

	cout << "Coefficients: " << endl;
	cout << _Cx << endl;
	cout << _Cy << endl;
	cout << _Cz << endl;

	_coefficientsLoaded = true;
	_polynomialOrder = _polynomialOrderForNewCalibration;

	// Save new values to ros parameter server
	saveCoefficients();
}
#endif

/*
 * saveCoefficients
 *
 * Save coefficients to calibration file
 * Used for polynomial fitting. Deprecated.
 */
#if 0
bool TrakstarCalibration::saveCoefficients() {

	XmlRpc::XmlRpcValue listCx, listCy, listCz;
	listCx.setSize(_Cx.size());
	listCy.setSize(_Cy.size());
	listCz.setSize(_Cz.size());
	for (unsigned int i = 0; i < _Cx.size(); i++) {
		listCx[i] = _Cx[i];
		listCy[i] = _Cy[i];
		listCz[i] = _Cz[i];
	}
	// Save coefficients
	ros::param::set("trakstar/calibration/internal/coefficients/Cx", listCx);
	ros::param::set("trakstar/calibration/internal/coefficients/Cy", listCy);
	ros::param::set("trakstar/calibration/internal/coefficients/Cz", listCz);

	// Save polynomial order
	ros::param::set("trakstar/calibration/internal/polynomial_order", (int)_polynomialOrder);

	return true;
}
#endif

/*
 * loadCoefficients
 *
 * Load coefficients from calibration file to internal storage _Cx, _Cy, _Cz
 * Used for Polynomial fitting. Deprecated.
 */
#if 0
bool TrakstarCalibration::loadCoefficients() {

	// Load calibration coefficients

	XmlRpc::XmlRpcValue listCx, listCy, listCz;

	if (!ros::param::has("trakstar/calibration/internal/coefficients/")) {
		cout << "TrakstarCalibration::loadCoefficients(): ros::param, parameters not available." << endl;
		return false;
	}
	ros::param::get("trakstar/calibration/internal/coefficients/Cx", listCx);
	ros::param::get("trakstar/calibration/internal/coefficients/Cy", listCy);
	ros::param::get("trakstar/calibration/internal/coefficients/Cz", listCz);
	ROS_ASSERT(listCx.getType() == XmlRpc::XmlRpcValue::TypeArray);
	ROS_ASSERT(listCy.getType() == XmlRpc::XmlRpcValue::TypeArray);
	ROS_ASSERT(listCz.getType() == XmlRpc::XmlRpcValue::TypeArray);
	_Cx.resize(listCx.size(), false);
	_Cy.resize(listCy.size(), false);
	_Cz.resize(listCz.size(), false);

	for (int i = 0; i < listCx.size(); i++)
	{
		ROS_ASSERT(listCx[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
		ROS_ASSERT(listCy[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
		ROS_ASSERT(listCz[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
		_Cx[i] = static_cast<double>(listCx[i]);
		_Cy[i] = static_cast<double>(listCy[i]);
		_Cz[i] = static_cast<double>(listCz[i]);
	}

	int polyOrder = 0;
	ros::param::get("trakstar/calibration/internal/polynomial_order", polyOrder);
	if (polyOrder > 0) {
		_polynomialOrder = polyOrder;
	} else {
		return false;
	}

	cout << "TrakstarCalibration: Loaded polynomial coefficients." << endl;

	_coefficientsLoaded = true;
	return true;
}
#endif

/*
 * getCalibratedPosition
 *
 * Get calibrated position based on Polynomial fitting. Deprecated
 */
#if 0
void TrakstarCalibration::getCalibratedPosition(double &x, double &y, double &z) {

	// No coefficients loaded. Cannot compute correction
	if (!_coefficientsLoaded) {
		cerr << "Cannot utilize getCalibratedPosition(x, y, z). No calibration values loaded. (Are they loaded into the ros parameter server?)" << endl;
		return;
	}

	double sumValx = 0;
	double sumValy = 0;
	double sumValz = 0;

	if (_p.size() == 0) {
		if (preparePermutations()) {
			cout << "prepared permutations!" << endl;
		} else {
			cerr << "Error. Did not prepare permutations... TrakstarCalibration error." << endl;
			return;
		}
	}

	stringstream sumStr1;
	stringstream sumStr2;
	sumStr1.clear();
	sumStr1 << "Sum1X: ";
	sumStr2.clear();
	sumStr2 << "Sum2X: ";
	for (unsigned int i = 0; i < _Cx.size(); i++) {
		sumValx += _Cx(i) * pow(x, _p[i][0]) * pow(y, _p[i][1]) * pow(z, _p[i][2]);
		sumValy += _Cy(i) * pow(x, _p[i][0]) * pow(y, _p[i][1]) * pow(z, _p[i][2]);
		sumValz += _Cz(i) * pow(x, _p[i][0]) * pow(y, _p[i][1]) * pow(z, _p[i][2]);
		sumStr1 << "+ (" << _Cx(i) << " * " << x << "^" << _p[i][0] << " * " << y << "^" << _p[i][1] << " * " << z << "^" << _p[i][2] << ") ";
		sumStr2 << "+ (" << _Cx(i) << " * " << pow(x, _p[i][0]) << " * " << pow(y, _p[i][1]) << " * " << pow(z, _p[i][2]) << ") ";
	}
	/*cout << sumStr1.str() << endl;
	 cout << sumStr2.str() << endl;
	 cout << "real values: x, y, z: " << x+sumValx << ", " << y+sumValy << ", " << z+sumValz << endl;*/
	x += sumValx;
	y += sumValy;
	z += sumValz;
}
#endif

/*
 * From main
 */
#if 0
_polynomialOrderForNewCalibration = newPolynomialOrder;
_polynomialOrder = 0;
_coefficientsLoaded = false;
#endif
#if 0
// Save coefficients from ros::param if available
// We must do this before preparing permutations, becase polynomialOrder might be updated when loading saved params
_coefficientsLoaded = loadCoefficients();

// Check if _polynomialOrder is sensible
if (_polynomialOrder == 0 || _coefficientsLoaded == false) {
	cout << "WARNING. TrakstarCalibration: Could not load calibration values from parameter server." << endl;
}

// Prepare permutations
preparePermutations();
#endif
