/**/
#ifndef UNIVERSALROBOTSSIM_HPP
#define UNIVERSALROBOTSSIM_HPP

#include "URServiceInterface.hpp"

#include <rw/kinematics/State.hpp>
#include <rw/models/SerialDevice.hpp>
#include <rw/invkin/JacobianIKSolver.hpp>
#include <rw/trajectory/Trajectory.hpp>
#include <rwlibs/algorithms/xqpcontroller/XQPController.hpp>


class UniversalRobotsSim: public URServiceInterface {
public:
	UniversalRobotsSim(int argc, char** argv, const std::string& name, rw::models::SerialDevice::Ptr device, const rw::kinematics::State& state);

protected:
	virtual bool moveL(const std::vector<rw::math::Transform3D<> >& targets, const std::vector<float>& blendValues, float speed);
	virtual bool moveQ(const std::vector<rw::math::Q>& targets, const std::vector<float>& blendValues, float speed);
	virtual bool safeMoveQ(const std::vector<rw::math::Q>& targets, const std::vector<float>& blendValues, float speed);
	virtual bool safeMoveL(const std::vector<rw::math::Transform3D<> >& targets, const std::vector<float>& blendValues, float speed);

	virtual bool servoT(const rw::math::Transform3D<>& target, const rw::math::VelocityScrew6D<>& velocity);

    virtual bool servoQ(const rw::math::Q& qtarget);


	virtual bool stop();
	virtual bool pause();
	virtual bool start();
	virtual bool wait();

	virtual void loop();
	virtual void stopDriver();

private:
	rw::models::SerialDevice::Ptr _device;
	rw::math::Q _q;
	rw::math::Q _dq;
	double _dt;
	rw::kinematics::State _state;
	rw::invkin::JacobianIKSolver _iksolver;
	rw::trajectory::QTrajectory::Ptr _qTrajectory;
	rw::trajectory::Transform3DTrajectory::Ptr _cartTrajectory;
	double _time;

	enum UROperationState { IDLE = 0, QTRAJECTORY, TTRAJECTORY, SERVO, SERVOQ };
	UROperationState _operationState;
	rwlibs::algorithms::XQPController _xqp;
	rw::math::Transform3D<> _servoTarget;
	rw::math::VelocityScrew6D<> _servoVelocity;
    rw::math::Q _servoQTarget;

	void servo();
    void servoQ();
};

#endif //#ifndef UNIVERSALROBOTSSIM_HPP
