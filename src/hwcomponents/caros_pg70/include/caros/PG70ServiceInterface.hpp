/**/
#ifndef PG70SERVICEINTERFACE_HPP
#define PG70SERVICEINTERFACE_HPP

#include <rw/math/Q.hpp>
#include <rw/math/Transform3D.hpp>

//#include "schunkpg70/Move.h"
//#include "schunkpg70/Open.h"
//#include "schunkpg70/Close.h"
//#include "schunkpg70/Home.h"
//#include "schunkpg70/Stop.h"

//#include "marvin_common/MarvinUtils.hpp"

//#include "marvin_common/ParallelGripperState.h"

#include "ros/ros.h"
#include <string>


class PG70ServiceInterface {
public:
	PG70ServiceInterface(const std::string& service_name, double loopRate);

	bool run();

protected:
//	MarvinTime _marvinTime;
	virtual bool move(float pos) = 0;
	virtual bool open(float force) = 0;
	virtual bool close(float force) = 0;
	virtual bool home() = 0;
	virtual bool stop() = 0;


	virtual void loop() = 0;
	virtual void stopDriver() = 0;


	double getLoopRate();
	//void publish(double q);
	//void publish(const marvin_common::ParallelGripperState& state);

private:

//	bool moveHandle(schunkpg70::Move::Request& request, schunkpg70::Move::Response& response);
//	bool openHandle(schunkpg70::Open::Request& request, schunkpg70::Open::Response& response);
//	bool closeHandle(schunkpg70::Close::Request& request, schunkpg70::Close::Response& response);
//	bool homeHandle(schunkpg70::Home::Request& request, schunkpg70::Home::Response& response);
//	bool stopHandle(schunkpg70::Stop::Request& request, schunkpg70::Stop::Response& response);

private:
    ros::NodeHandle _nodeHnd;

    ros::Publisher _statePublisher;
    ros::ServiceServer _srvMove;
    ros::ServiceServer _srvOpen;
    ros::ServiceServer _srvClose;
    ros::ServiceServer _srvHome;
    ros::ServiceServer _srvStop;

    std::string _service_name;
    ros::Rate _loopRate;

};

#endif //#ifndef PG70SERVICEINTERFACE_HPP
