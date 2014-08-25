// Copyright (c) 2010, University of Southern Denmark
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the University of Southern Denmark nor the names of
//    its contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE UNIVERSITY OF SOUTHERN DENMARK BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/**
  \author liljekrans
  \file Trakstar.hpp
  \date Oct 27, 2011
  \brief 
 */
 

#ifndef TRAKSTAR_HPP_
#define TRAKSTAR_HPP_


#include <ros/ros.h>
#include "ATC3DG.h"
//#include "TrakstarHelper.hpp"

#include <boost/bind.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>

#include <rw/math/Vector3D.hpp>
#include <rw/math/Quaternion.hpp>


#include <vector>

#define TRAKSTAR_DATA_RECORD_SIZE			8

#define TRAKSTAR_STATUS_STOPPED				-1
#define TRAKSTAR_STATUS_INITIALIZING		0
#define TRAKSTAR_STATUS_STARTED				1

// Change both when changin
//typedef DOUBLE_POSITION_QUATERNION_TIME_Q_BUTTON_RECORD TRAKSTAR_RECORDS_TYPE;
//#define TRAKSTAR_RECORDS_ENUM_TYPE 		DOUBLE_POSITION_QUATERNION_TIME_Q_BUTTON

//typedef DOUBLE_POSITION_MATRIX_TIME_Q_BUTTON_RECORD TRAKSTAR_RECORDS_TYPE;
//#define TRAKSTAR_RECORDS_ENUM_TYPE 		DOUBLE_POSITION_MATRIX_TIME_Q_BUTTON

// Get all information possible
typedef DOUBLE_POSITION_ANGLES_MATRIX_QUATERNION_TIME_Q_BUTTON_RECORD TRAKSTAR_RECORDS_TYPE;
#define TRAKSTAR_RECORDS_ENUM_TYPE 	DOUBLE_POSITION_ANGLES_MATRIX_QUATERNION_TIME_Q_BUTTON;

class Trakstar {

public:
	struct PoseData {
		rw::math::Vector3D<> 	pos;
		rw::math::Quaternion<> 	rot;
		double 					time;
		double 					quality; // quality from 0 to 1
		bool 					valid;
		ULONG 					status;
		bool		 			calibStatus;
		bool					analogButtonOn;
	};

private:
	SYSTEM_CONFIGURATION      				_ATC3DG;   // a pointer to a single instance of the system class
	std::vector<SENSOR_CONFIGURATION> 		_pSensor;  // a pointer to an array of sensor objects
	std::vector<TRANSMITTER_CONFIGURATION> 	_pXmtr;    // a pointer to an array of transmitter objects
	std::vector<TRAKSTAR_RECORDS_TYPE> 		_rawValues;

	std::vector<PoseData>					_records, _recordsTmp;

	int				_errorCode;			// used to hold error code returned from procedure call
	
	int 			_sensorsAttached;	// Is updated with the actual number of connected sensors at startPolling()
	
	boost::thread 	_initThread;
	boost::thread 	_pollThread;
	bool			_flagStopPoll;
	bool			_analogButtonOn;
	
	boost::mutex 	_mutexSensorValues;
	int 			_initStatus;
	
	int InitializeBird();
	void pollData();
	
	
public:
	Trakstar();
	~Trakstar();
	
	void InitializeSystem(bool block = true);
	int getInitStatus();
		
	std::vector<PoseData> getData(void);
	bool startPolling();
	void stopPolling();
	bool isPolling() { return !_flagStopPoll; }
	
	int getNumberSensorsAttached();
	int getNumberSensorsSupported() { return _ATC3DG.numberSensors; }
	
	std::string getSensorStatusString(int errorcode);
	
private:
	void publish();


	ros::Publisher state_publiser_;

};





#endif /* TRAKSTAR_HPP_ */
