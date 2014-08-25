#include "UniversalRobots.hpp"

#include <marvin_common_rw/RwRos.hpp>
#include <rw/invkin/IKMetaSolver.hpp>
#include <rw/math/MetricFactory.hpp>
#include <rw/math/Wrench6D.hpp>
#include <rwhw/netft/NetFTCommon.hpp>


using namespace rwhw;
using namespace rw::math;
using namespace rw::common;
using namespace rw::models;
using namespace rw::kinematics;
using namespace rw::invkin;
using namespace rwlibs::algorithms;

UniversalRobots::UniversalRobots(WorkCell::Ptr workcell,
								 const PropertyMap& properties,
								 int looprate,
								 const std::string& wrenchTopic,
								 rwhw::FTCompensation::Ptr pFTCompensation):
	SerialDeviceServiceInterface(properties.get<std::string>("Name")),
	_loopRate(looprate),
	_workcell(workcell),
	_device(NULL),
	_ftFrame(NULL),
	_properties(properties),
	_pFTCompensation(pFTCompensation),
	_useFTCollisionDetection(false)
{

	//_srvMoveLinFC = _nodeHnd->advertiseService("servoT", &UniversalRobots::servoHandle, this);

    _srvServo = _nodeHnd->advertiseService("servo", &UniversalRobots::servoHandle, this);
    _srvServoQ = _nodeHnd->advertiseService("servoq", &UniversalRobots::servoQHandle, this);

    _srvForceModeStart = _nodeHnd->advertiseService("force_mode_start", &UniversalRobots::forceControlStart, this);
    _srvForceModeUpdate = _nodeHnd->advertiseService("force_mode_update", &UniversalRobots::forceControlUpdate, this);
    _srvForceModeStop = _nodeHnd->advertiseService("force_mode_stop", &UniversalRobots::forceControlStop, this);


	if (_workcell != NULL) {
		_device = _workcell->findDevice( properties.get<std::string>("Name","") );
		if (_device == NULL) {
			RW_THROW("Unable to find device: "<<properties.get<std::string>("Name","")<<" in workcell");
		}
		_dt = 1.0/(double)looprate;
	//	_xqp = ownedPtr(new XQPController(_device, _device->getEnd(), workcell->getDefaultState(), _dt));
		if (properties.has("FTFrame")) {
			_ftFrame = _workcell->findFrame(properties.get<std::string>("FTFrame","WORLD"));
		}
	}

	_state = _workcell->getDefaultState();
	_iksolver = ownedPtr( new JacobianIKSolver(_device, _state ) );

	std::string ip = _properties.get<std::string>("IP","");
	if( ip.empty() ){
		ROS_ERROR("IP Adress is not setup correctly!");
		ROS_BREAK();
	}

	int port = _properties.get<int>("Port",33333);

	_urrt.connect(ip, 30003);
	//std::cout<<"Transfer Script: "<<scriptFile<<std::endl;
	_ur.connect(ip, 30001);

	_ur.startInterface(port);
	_urrt.start();


	if (wrenchTopic != "") {
		subFTData = _nodeHnd->subscribe(wrenchTopic, 1000, &UniversalRobots::addFTData, this);
	}

/*	if (_pNetFT != NULL) {
		_pNetFT->start();
	}
*/
    Q weights(6);
    weights(0) = 0.85;
    weights(1) = 0.85;
    weights(2) = 0.45;
    weights(3) = 0.30;
    weights(4) = 0.20;
    weights(5) = 0.20;
    _q2cmetric = MetricFactory::makeWeightedEuclidean(weights);

}

void UniversalRobots::stopDriver() {

	_ur.stopInterface();
	_urrt.stop();
}

void UniversalRobots::addFTData(const marvin_common::WrenchData::ConstPtr& state) {
	rwhw::Wrench3D wrench;

	wrench.first(0) = state->wrench.force.x;
	wrench.first(1) = state->wrench.force.y;
	wrench.first(2) = state->wrench.force.z;
	wrench.second(0) = state->wrench.torque.x;
	wrench.second(1) = state->wrench.torque.y;
	wrench.second(2) = state->wrench.torque.z;

	if (_wrenchDataQueue.size() >= 3) {
		_wrenchDataQueue.pop();
	}

	_wrenchDataQueue.push(wrench);
}

bool UniversalRobots::updateFTBias() {
	URRTData urData = _urrt.getLastData();

	if (urData.qActual.size() != 6) {
		ROS_ERROR("Unable to get configuration from UR");
		return false;
	}

	std::vector<rwhw::Wrench3D> bias;
	for (size_t i = 0; i<3; i++) {
		bias.push_back(_wrenchDataQueue.front());
		_wrenchDataQueue.pop();
	}
//		_pNetFT->waitForNewData();
//		NetFTLogging::NetFTData ftdata = _pNetFT->getAllData();
//		_pFTCompensation->update(ftdata.data, urData.qActual);
//		bias.push_back(_pFTCompensation->getFT());
	//	std::cout<<"Bias = "<<bias.back().first<<" "<<bias.back().first<<std::endl;
	if(_pFTCompensation)
		_pFTCompensation->updateBias(bias);

	return true;
}

void UniversalRobots::loop() {

	if (_urrt.hasData() == false || _ur.getPrimaryInterface().hasData() == false) {
		ROS_WARN_STREAM("Waiting for data from UR");
		return;
	}

	URRTData urData = _urrt.getLastData();
	UniversalRobotsData purData = _ur.getPrimaryInterface().getLastData();

	std::cout<<"\rE-stop: "<<purData.emergencyStopped<<" Security-Stop: "<<purData.securityStopped<<" Running: "<<purData.programRunning;
	if (_ur.getPrimaryInterface().getMessages().size() > 0) {
		std::cout<<std::endl;
		std::queue<URMessage> messages = _ur.getPrimaryInterface().getMessages();
		while (messages.size() > 0) {
			std::cout<<messages.front()<<std::endl;
			messages.pop();
		}
		_ur.getPrimaryInterface().clearMessages();

	}
	//if(_ur.getPrimaryInterface()._lostConnection)
	/*
	ROS_INFO_STREAM("urstamp:" << _ur.getPrimaryInterface()._lastPackageTime;);
	ROS_INFO_STREAM("urrtstamp:" << _urrt._lastPackageTime;);

	if(purData.speedFraction<0.9)
		ROS_INFO_STREAM("speed: " << purData.speedFraction);
	if( purData.emergencyStopped>0 )
		ROS_INFO_STREAM("estop: " << purData.emergencyStopped);
	if( purData.securityStopped>0 )
		ROS_INFO_STREAM("sstop: " << purData.securityStopped);
	 */
	bool isColliding = false;

    //boost::mutex::scoped_lock lock(_mutex);



/*
	if (//_pNetFT != NULL &&
			_pFTCompensation != NULL) {
//		NetFTLogging::NetFTData ftdata = _pNetFT->getAllData();
//		std::pair<Vector3D<>, Vector3D<> > wrench = ftdata.data;
		rwhw::Wrench3D wrench = _wrenchDataQueue.back();
		_pFTCompensation->update(wrench, urData.qActual);


		if (_useFTCollisionDetection && _pFTCompensation != NULL && urData.qActual.size() == 6) {
			if (_pFTCompensation->inCollision()) {
				isColliding = true;
				std::cout<<"Stop due to collision "<<std::endl;
				std::pair<Vector3D<>, Vector3D<> > w = _pFTCompensation->getFT();
				std::cout<<"Wrench = "<<w.first<<" "<<w.second<<std::endl;
				_errorMsg = "Stopped due to collision";
				_ur.stopRobot();
			}
		}
	}
*/




	if (urData.qActual.size() == 6) {
		marvin_common::RobotState state;
		_qcurrent = urData.qActual;
		state.q = RwRos::toRos(urData.qActual);
		state.dq = RwRos::toRos(urData.dqActual);
		state.header.frame_id = _nodeHnd->getNamespace();
        state.header.stamp = ros::Time::now();
		state.estopped = purData.emergencyStopped;
		if(state.estopped){
			//ROS_WARN("Robot is in emergency stop!"); // this data is currently broken...
		}
		state.isMoving = _ur.isMoving();
		state.isColliding = isColliding;
		publish(state);

/*
		marvin_common::WrenchData wrench;
		wrench.header.frame_id = name();
        wrench.header.stamp = ros::Time::now();
		wrench.wrench.force.x = urData.tcpForce(0);
		wrench.wrench.force.y = urData.tcpForce(1);
		wrench.wrench.force.z = urData.tcpForce(2);
		wrench.wrench.torque.x = urData.tcpForce(3);
		wrench.wrench.torque.y = urData.tcpForce(4);
		wrench.wrench.torque.z = urData.tcpForce(5);
		publish(wrench);
*/
	}

    //_urrt.update();
}




bool UniversalRobots::movePTP(marvin_common::SerialDeviceMovePTP::Request& request,
							  marvin_common::SerialDeviceMovePTP::Response& response)
{
	//ROS_INFO("Start movePTP");
	boost::mutex::scoped_lock lock(_mutex);
	for (size_t i = 0; i<request.targets.size(); i++) {
		double speed = 100;
		if(request.speeds.size()<i)
			speed = request.speeds[i];
		_ur.moveQ(RwRos::toRw(request.targets[i]), speed);
	}
	return true;

}

bool UniversalRobots::movePTP_T(marvin_common::SerialDeviceMovePTP_T::Request& request,
				  				marvin_common::SerialDeviceMovePTP_T::Response& response)
{
	boost::mutex::scoped_lock lock(_mutex);
	for (size_t i = 0; i<request.targets.size(); i++) {
		_device->setQ(_qcurrent, _state);
		Transform3D<> target = RwRos::toRw(request.targets[i]);
		std::vector<Q> solutions = _iksolver->solve(target, _state);
		if (solutions.size() == 0) {
	        ROS_ERROR_STREAM("movePTP_T: Unable to find IK solution for: " << target << _qcurrent);
			return false;
		}
		_ur.moveQ(solutions.front(), request.speeds[i]);
	}
	return true;
}

bool UniversalRobots::servoQ(marvin_common::SerialDeviceMovePTP::Request& request,
							  marvin_common::SerialDeviceMovePTP::Response& response)
{
    ROS_DEBUG_STREAM("servoq ");
	//ROS_INFO("Start movePTP");
	if(request.targets.size()>0){
		rw::math::Q q = RwRos::toRw(request.targets[0]);
		return servoQ( q );
	}
	return false;

}

bool UniversalRobots::servoT(marvin_common::SerialDeviceMovePTP_T::Request& request,
				  				marvin_common::SerialDeviceMovePTP_T::Response& response)
{
  ROS_DEBUG_STREAM("servoT ");
	if(request.targets.size()>0){
	  rw::math::Transform3D<> trans = RwRos::toRw(request.targets[0]);
	  return servoT(trans  );
	}
	return false;
}


bool UniversalRobots::servoT(const rw::math::Transform3D<>& target) {
  //boost::mutex::scoped_lock lock(_mutex);
  ROS_DEBUG_STREAM("servoT");
  _device->setQ(_qcurrent, _state);
  std::vector < Q > solutions = _iksolver->solve(target, _state);
  if (solutions.size() == 0)
  {
    ROS_ERROR_STREAM("servoT: Unable to find IK solution for: " << target << _qcurrent);
    return true;
  }

  Q closest = solutions.front();

/*	double dist = (qcurrent-closest).norm2();
	for (size_t i = 1; i<solutions.size(); i++) {
		double d = (qcurrent-solutions[i]).norm2();
		if (d < dist) {
			closest = solutions[i];
			dist = d;
		}
	}
  */
   /* if (_q2cmetric->distance(closest, qcurrent) > 0.1) {
        ROS_WARN("Servo target too far away");
        return true;
    }
*/
	//std::cout<<"Servo Q Goal = "<<closest<<std::endl;
    ROS_DEBUG_STREAM("driver servoq: " << closest);
	_ur.servo(closest);

//    ROS_INFO("Finished ServoT");
	return true;
}



bool UniversalRobots::moveQ(const std::vector<rw::math::Q>& targets, const std::vector<float>& blendValues, float speed)
{
	//boost::mutex::scoped_lock lock(_mutex);
	_useFTCollisionDetection = false;

	ROS_INFO("MoveQ %i", (int)targets.size());
	BOOST_FOREACH(const rw::math::Q& q, targets) {
		_ur.moveQ(q, speed);
	}
	return true;
}


bool UniversalRobots::safeMoveQ(const std::vector<rw::math::Q>& targets, const std::vector<float>& blendValues, float speed) {
	//boost::mutex::scoped_lock lock(_mutex);
	if (_pFTCompensation == NULL /*|| _pNetFT == NULL*/) {
		ROS_ERROR("Unable to perform motion with collision detection. No Force/Torque sensor available!");
		return false;
	}

	_useFTCollisionDetection = true;


	ROS_INFO("SafeMoveQ %i", (int)targets.size());

	if (!updateFTBias()) {
		ROS_ERROR("Unable to calibrate FT bias. Motion dropped.");
		return false;
	}

	BOOST_FOREACH(const rw::math::Q& q, targets) {
		_ur.moveQ(q, speed);
	}
	return true;
}


bool UniversalRobots::moveLin(marvin_common::SerialDeviceMoveLin::Request& request,
		  				     marvin_common::SerialDeviceMoveLin::Response& response)
{
  ROS_DEBUG_STREAM("moveLin");
	//boost::mutex::scoped_lock lock(_mutex);
	for (size_t i = 0; i<request.targets.size(); i++) {
		Transform3D<> target = RwRos::toRw(request.targets[i]);
		float speed = request.speeds[i];
		_ur.moveT(target, speed);
	}
	return true;
}

bool UniversalRobots::moveL(const std::vector<rw::math::Transform3D<> >& targets, const std::vector<float>& blendValues, float speed)
{
  ROS_DEBUG_STREAM("moveL");
	//boost::mutex::scoped_lock lock(_mutex);
	_useFTCollisionDetection = false;

	ROS_INFO("MoveT %i", (int)targets.size());
	BOOST_FOREACH(const rw::math::Transform3D<>& transform, targets) {
		_ur.moveT(transform, speed);
	}
	return true;
}


bool UniversalRobots::safeMoveL(const std::vector<rw::math::Transform3D<> >& targets, const std::vector<float>& blendValues, float speed) {
	boost::mutex::scoped_lock lock(_mutex);
	if (_pFTCompensation == NULL /*|| _pNetFT == NULL*/) {
		ROS_ERROR("Unable to perform motion with collision detection. No Force/Torque sensor available!");
		return false;
	}

	_useFTCollisionDetection = true;


	ROS_INFO("SafeMoveT %i", (int)targets.size());

	if (!updateFTBias()) {
		ROS_ERROR("Unable to calibrate FT bias. Motion dropped.");
		return false;
	}

	BOOST_FOREACH(const rw::math::Transform3D<>& transform, targets) {
		_ur.moveT(transform, speed);
	}
	return true;
}


bool UniversalRobots::servoQHandle(marvin_common::URServoQ::Request& request, marvin_common::URServoQ::Response& response) {
	ROS_DEBUG("URServiceInterface::servoQHandle");
	Q qtarget = RwRos::toRw(request.qtarget);

	return servoQ(qtarget);

}

bool UniversalRobots::servoQ(const rw::math::Q& qtarget) {
    ROS_DEBUG_STREAM("ServoQ: "<<qtarget);
    _ur.servo(qtarget);
    return true;
}

bool UniversalRobots::moveVelQ(marvin_common::SerialDeviceMoveVelQ::Request& request,
			   marvin_common::SerialDeviceMoveVelQ::Response& response)
{
	Q q_vel = RwRos::toRw(request.q_vel);
	Q qtarget = _qcurrent + q_vel*0.1;
	servoQ(qtarget);
	return true;
}

bool UniversalRobots::moveVelT(marvin_common::SerialDeviceMoveVelT::Request& request,
			   marvin_common::SerialDeviceMoveVelT::Response& response)
{
	boost::mutex::scoped_lock lock(_mutex);
	VelocityScrew6D<> t_vel = RwRos::toRw(request.vel);
	_device->setQ(_qcurrent, _state);
	Jacobian jac = _device->baseJend(_state);
	Jacobian jacInv( LinearAlgebra::pseudoInverseEigen(jac.e()) );

	Q qtarget = _qcurrent + (jacInv*t_vel)*0.1;
	servoQ(qtarget);
	return true;

}



bool UniversalRobots::servoHandle(marvin_common::URServo::Request& request, marvin_common::URServo::Response& response) {
    std::cout<<"Delay = "<<ros::Time::now() - request.stamp<<std::endl;
	Transform3D<> target = RwRos::toRw(request.target);

	return servoT(target);
}


bool UniversalRobots::forceControlStart(marvin_common::SerialDeviceForceControlStart::Request& request,
	   	    						 marvin_common::SerialDeviceForceControlStart::Response& response)
{
	std::cout<<"Received force command"<<std::endl;
	boost::mutex::scoped_lock lock(_mutex);

	Transform3D<> refToffset = RwRos::toRw(request.base2forceFrame);

	rw::math::Wrench6D<> wrenchTarget;
	wrenchTarget(0) = request.wrench.force.x;
	wrenchTarget(1) = request.wrench.force.y;
	wrenchTarget(2) = request.wrench.force.z;

	wrenchTarget(3) = request.wrench.torque.x;
	wrenchTarget(4) = request.wrench.torque.y;
	wrenchTarget(5) = request.wrench.torque.z;

	Q selection(Q::zero(6));
	for (size_t i = 0; i<request.selection.size(); i++) {
		selection(i) = request.selection[i];

	}

	Q limits(6);
	for (size_t i = 0; i<request.limits.size(); i++) {
		limits(i) = request.limits[i];
	}
	std::cout<<"Call rwhw::URCallBackInterface"<<std::endl;
	_ur.forceModeStart(refToffset, selection, wrenchTarget, limits);
	return true;
}

bool UniversalRobots::forceControlUpdate(marvin_common::SerialDeviceForceControlUpdate::Request& request,
			 marvin_common::SerialDeviceForceControlUpdate::Response& response)
{
	boost::mutex::scoped_lock lock(_mutex);

	rw::math::Wrench6D<> wrenchTarget;
	wrenchTarget(0) = request.wrench.force.x;
	wrenchTarget(1) = request.wrench.force.y;
	wrenchTarget(2) = request.wrench.force.z;

	wrenchTarget(3) = request.wrench.torque.x;
	wrenchTarget(4) = request.wrench.torque.y;
	wrenchTarget(5) = request.wrench.torque.z;

	std::cout<<"Call rwhw::URCallBackInterface::forceModeUpdate "<<wrenchTarget<<std::endl;
	_ur.forceModeUpdate(wrenchTarget);
	return true;
}

bool UniversalRobots::forceControlStop(marvin_common::SerialDeviceForceControlStop::Request& request,
		 marvin_common::SerialDeviceForceControlStop::Response& response)
{
	boost::mutex::scoped_lock lock(_mutex);

	_ur.forceModeEnd();
	return true;
}


bool UniversalRobots::moveLinFC(marvin_common::SerialDeviceMoveLinFC::Request& request,
			     	   	   	    marvin_common::SerialDeviceMoveLinFC::Response& response)
{
	boost::mutex::scoped_lock lock(_mutex);

	if (_ftFrame == NULL) {
		_errorMsg = "Unable to use force command without having defined a frame of the FT sensor";
		RW_WARN("Unable to use force command without having defined a frame of the FT sensor");
		return false;
	}

	State state = _workcell->getDefaultState();
	_device->setQ(_qcurrent, state);

	Transform3D<> refTtarget = RwRos::toRw(request.pos_target);
	Transform3D<> refToffset = RwRos::toRw(request.offset);
	Transform3D<> baseTref = Transform3D<>::identity();
	Transform3D<> baseTtarget = baseTref*refTtarget;
	Transform3D<> baseToffset = baseTref*refToffset;
	Transform3D<> base2ft = _device->baseTframe(_ftFrame, state);
	Transform3D<> baseTend = _device->baseTend(state);
	Transform3D<> endTtarget = inverse(baseTend)*baseTtarget;
	Transform3D<> endToffset = inverse(baseTend)*baseToffset;
	Transform3D<> ftToffset = inverse(base2ft)*baseToffset;

	rwhw::Wrench3D wrenchTarget;
	wrenchTarget.first(0) = request.wrench_target.force.x;
	wrenchTarget.first(1) = request.wrench_target.force.y;
	wrenchTarget.first(2) = request.wrench_target.force.z;

	wrenchTarget.second(0) = request.wrench_target.torque.x;
	wrenchTarget.second(1) = request.wrench_target.torque.y;
	wrenchTarget.second(2) = request.wrench_target.torque.z;

	Q selection(6);
	for (size_t i = 0; i<6; i++) {
		selection(i) = request.ctrl_gains[i];
	}

	rwhw::Wrench3D wrenchCurrent = _wrenchDataQueue.back();
	wrenchCurrent.first = ftToffset.R() * wrenchCurrent.first;
	wrenchCurrent.second = ftToffset.R() * wrenchCurrent.second;

	rwhw::Wrench3D wrenchDiff (wrenchTarget.first - wrenchCurrent.first, wrenchTarget.second - wrenchCurrent.second);


	Vector3D<> p_offset = endToffset.R() * endTtarget.P();
	EAA<> eaa_offset = endToffset.R() * EAA<>(endTtarget.R());
	for (size_t i = 0; i<3; i++) {
		if (selection(i) != 0)
			p_offset(i) = 0;
		else
			p_offset(i) = selection(i)*wrenchDiff.first(i);
		if (selection(i+3) != 0)
			eaa_offset(i) = 0;
		else
			eaa_offset(i) = selection(i)*wrenchDiff.second(i);
	}

	endToffset = Transform3D<>(p_offset, eaa_offset);

	Transform3D<> baseTtarget2 = baseTend*endToffset;
	return servoT(baseTtarget2);

}



bool UniversalRobots::stop(marvin_common::Stop::Request& request,
						   marvin_common::Stop::Response& response)
{
	_ur.stopRobot();
	return true;
};



bool UniversalRobots::run() {

	  while (ros::ok())
	  {
		  loop();
		  ros::spinOnce();
		  _loopRate.sleep();
	  }
	  stopDriver();
	  return true;
}



