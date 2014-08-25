/**/
#include "UniversalRobotsSim.hpp"

#include <rw/math/MetricFactory.hpp>
#include <rw/trajectory/TrajectoryFactory.hpp>

using namespace rw::math;
using namespace rw::kinematics;
using namespace rw::models;
using namespace rw::trajectory;
using namespace rwlibs::algorithms;

namespace {
const int LOOPRATE = 100;
}

UniversalRobotsSim::UniversalRobotsSim(int argc, char** argv, const std::string& name, SerialDevice::Ptr device, const State& state):
	URServiceInterface(name, LOOPRATE, true),
	_device(device),
	_dt(1.0/LOOPRATE),
	_state(state),
	_iksolver(device, state),
	_operationState(IDLE),
	_xqp(device, device->getEnd(), state, _dt)
{
	_q = _device->getQ(_state);
	_dq = Q::zero(_q.size());
}

void UniversalRobotsSim::stopDriver() {

}

void UniversalRobotsSim::loop()
{
	//Q qnew = _device->getQ(_state);
	switch (_operationState) {
	case IDLE:
		break;
	case QTRAJECTORY:
		if (_qTrajectory != NULL) {
			if (_time < _qTrajectory->duration()) {
				_q = _qTrajectory->x(_time);
				_dq = _qTrajectory->dx(_time);
			}
			else {
				_q = _qTrajectory->x(_qTrajectory->endTime());
				_dq = _qTrajectory->dx(_qTrajectory->endTime());
			}
		}
		break;
	case TTRAJECTORY:
		if (_cartTrajectory != NULL) {
			Transform3D<> transform = _cartTrajectory->x(_time);
			std::vector<Q> qs = _iksolver.solve(transform, _state);
			if (qs.size() > 0) {
				_dq = (qs.front() - _q)/(1.0/getLoopRate());
				_q = qs.front();
			}
		}
	case SERVO:
		std::cout<<"Servo in loop"<<std::endl;
		servo();
		break;
    case SERVOQ:
        std::cout<<"ServoQ in loop"<<std::endl;
        servoQ();
        break;
	}
	//std::cout<<"qnew =? "<<qnew<<std::endl;
	_device->setQ(_q, _state);
	publish(_q);

	_time += getLoopRate();

}

bool UniversalRobotsSim::moveL(const std::vector<rw::math::Transform3D<> >& targets, const std::vector<float>& blendValues, float speed)
{
	_cartTrajectory = TrajectoryFactory::makeLinearTrajectory(targets, MetricFactory::makeTransform3DMetric<double>(1,1));
	_qTrajectory = NULL;
	_time = 0;
	return true;
}

bool UniversalRobotsSim::moveQ(const std::vector<rw::math::Q>& targets, const std::vector<float>& blendValues, float speed)
{
	QPath path;
	std::cout<<"Target Count "<<targets.size()<<std::endl;
	BOOST_FOREACH(const Q& q, targets) {
		std::cout<<"Q = "<<q<<std::endl;
	}
	Q qcurrent = _device->getQ(_state);
	path.push_back(qcurrent);
	path.insert(path.end(), targets.begin(), targets.end());
	_qTrajectory = TrajectoryFactory::makeLinearTrajectory(path, MetricFactory::makeEuclidean<Q>());
	_cartTrajectory = NULL;
	_time = 0;
	return true;
}

bool UniversalRobotsSim::safeMoveQ(const std::vector<rw::math::Q>& targets, const std::vector<float>& blendValues, float speed) {
	return false;
}


bool UniversalRobotsSim::safeMoveL(const std::vector<rw::math::Transform3D<> >& targets, const std::vector<float>& blendValues, float speed) {
	return false;
}

bool UniversalRobotsSim::servoQ(const rw::math::Q& qtarget) {
    std::cout<<"UniversalRobotsSum::servoQ: "<<qtarget<<std::endl;
    _servoQTarget = qtarget;
    _operationState = SERVOQ;
    return true;
}

bool UniversalRobotsSim::servoT(const rw::math::Transform3D<>& Ttarget, const rw::math::VelocityScrew6D<>& velocity) {
	std::cout<<"UniversalRobotsSim::servo: "<<Ttarget<<" "<<velocity<<std::endl;

	_servoTarget  = Ttarget;
	_servoVelocity = velocity;
	_operationState = SERVO;
	return true;
}

void UniversalRobotsSim::servoQ() {
    Q qcurrent = _q;
    Q dq = _servoQTarget-qcurrent;
    _q += 0.1*dq;   
}

void UniversalRobotsSim::servo() {
	_device->setQ(_q, _state);
    Frame* tcpFrame = _device->getEnd();
    Transform3D<> Tcurrent = _device->baseTframe(tcpFrame, _state);
    Transform3D<> Tdiff = inverse(Tcurrent)*_servoTarget;


/*    if (!safe(_q)) {
        log(Info)<<"Configuration in collision. Use Jog to move to a safe configuration"<<endlog();
        _portDQTarget.Set(Q::zero(_device->getDOF()));
        return;
    }
*/

    VelocityScrew6D<> vs(Tdiff);
    //log(Info)<<"Error = "<<vs<<endlog();
    double gain = 0.1;
    VelocityScrew6D<> diff = gain*(Tcurrent.R()*vs);
    diff += _servoVelocity;

    double linvel = diff.linear().norm2();

    const double maxLinearVelocity = 0.5;
    if (linvel > maxLinearVelocity) {
        diff *= maxLinearVelocity/linvel;
    }

    const double maxAngularVelocity = 0.5;
    if (diff.angular().angle() > maxAngularVelocity) {
        diff *= maxAngularVelocity/diff.angular().angle();
    }

    //log(Info)<<"Call PseudoInverse"<<endlog();
    Q dqtarget = _xqp.solve(_q, _dq, diff, std::list<XQPController::Constraint>());
    _dq = dqtarget;
    _q += _dt*_dq;
    std::cout<<"Qnew = "<<_q<<std::endl;

}

bool UniversalRobotsSim::stop()
{
	_qTrajectory = NULL;
	_cartTrajectory = NULL;
	return true;
}


bool UniversalRobotsSim::pause()
{
	return true;
}

bool UniversalRobotsSim::start()
{
	return true;
}

bool UniversalRobotsSim::wait()
{
	return true;
}
