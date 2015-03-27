#include <caros/PG70Node.hpp>

#include <rw/math/MetricUtil.hpp>

using namespace rwhw;
using namespace rw::math;
using namespace rw::common;

PG70Node::PG70Node(const ros::NodeHandle& nodehandle):
    caros::CarosNodeServiceInterface(nodehandle),
    caros::GripperServiceInterface(nodehandle),
    _nh(nodehandle),
    _pg70(0)
{

}

PG70Node::~PG70Node(){
   if (_pg70 != 0) {
        if (_pg70->isConnected()) {
            ROS_DEBUG_STREAM("Still connected to the PG70 device - going to stop the device and disconnect.");
            _pg70->stop();
            _pg70->disconnect();
        }
        delete _pg70;
        _pg70 = 0;
    }
}

bool PG70Node::configureHook() {
   if (_pg70 != 0) {
        CAROS_FATALERROR("The PG70 device is already active - please properly cleanup before trying to configure the PG70.", PG70NODE_SDH_DEVICE_ALREADY_ACTIVE);
        return false;
    }
    _pg70 = new rwhw::SchunkPG70;

    /* Fetch parameters (if any) or use the defaults */
    _nh.param("port", _port, std::string("/dev/ttyS4"));
    _nh.param("name", _name, std::string("PG70"));
    
    _pg70->connectSerial(_port);
    
     /* TODO: Verify that the chosen interfaceType is valid? or just let it fail when start is invoked? */

    /* TODO: Could make the use of the gripper service, configurable through the parameter server. */
    if (!configureGripperService()) {
        CAROS_FATALERROR("The CAROS GripperService could not be configured correctly.", PG70NODE_CAROS_GRIPPER_SERVICE_CONFIGURE_FAIL);
        return false;
    }

    /* Outputting information on supported value ranges */
    /* TODO: This could be made part of the GripperServiceInterface - possibly as a message that is returned (or published) when a client asks for it.
     * If the hardware is intelligent enough to provide new values/boundaries according to position or grasping mode, then it could make sense to publish that information when it changes
     */

    /* TODO: Debug information on what was configured accoringly to the parameter server? */
    return true;
  
  
}

bool PG70Node::cleanupHook() {
    /* Keep cleaning up even if the gripper service can't be cleaned up properly */
    bool retVal = true;
    if (!cleanupGripperService()) {
        retVal = false;
        ROS_ERROR_STREAM("cleanupGripperService() failed.");
    }

    ROS_WARN_STREAM_COND(_pg70 == 0, "cleanupHook() was called with the PG70 device not being configured (i.e. no valid PG70-object)"); 
    if (_pg70 != 0) {
        if (_pg70->isConnected()) {
            ROS_DEBUG_STREAM("Still connected to the PG70 device - going to stop the device and disconnect.");
            _pg70->stop();
            _pg70->disconnect();
        }
        delete _pg70;
        _pg70 = 0;
    }

    return retVal;
}

bool PG70Node::startHook() {
    if (_pg70 == 0) {
        CAROS_FATALERROR("The PG70 device is not configured", PG70NODE_INTERNAL_ERROR);
        return false;
    }

    if (_pg70->isConnected()) {
        /* Not jumping to an error state, thus allowing further use after having startHook() mistakenly invoked.
         * This can happen for a variety of reasons, of which some are:
         * - A "misconfiguration" - some other node just invoking start on this node without verifying that this node is not currently running
         * - A race condition (multiple nodes trying to start this node - although highly unlikely when the callback queues are processed one by one)
         */
        ROS_ERROR_STREAM("'" << __PRETTY_FUNCTION__ << "' invoked even though a connection to the PG70 device has already been established - this should be considered a bug!"); 
        return false;
    }

    /* Connect according to interface type and configured parameters */
    if (!_port.empty()) {
       _pg70->connectSerial(_port);
    }
    else {
        CAROS_FATALERROR("No nome of the serial port provided!", PG70NODE_UNSUPPORTED_INTERFACE_TYPE);
        return false;
    }

    /* Verify that the connection to the SDH has been established - this eliminates the need for verifying the _sdh->connect() function calls actually succeeded */
    if (! _pg70->isConnected()) {
        /* Something went wrong when connecting */
        CAROS_FATALERROR("Failed to properly connect to the PG70 device.", PG70NODE_PG70_DEVICE_CONNECT_FAILED);
        return false;
    }

    return true;
}


bool PG70Node::stopHook() {
    if (_pg70 == 0) {
        CAROS_FATALERROR("The PG70 device is not configured", PG70NODE_INTERNAL_ERROR);
        return false;
    } else {
        if (_pg70->isConnected()) {
            /* TODO: calling stop() is probably unnecessary */
            _pg70->stop();
            _pg70->disconnect();
        } else {
            ROS_WARN_STREAM("There was no established connection to the PG70 device when '" << __PRETTY_FUNCTION__ << "' was invoked! - Consider this a bug!");
        }
    }

    return true;
}

bool PG70Node::recoverHook() {
/* TODO: */
    /* Maybe the connection to the SDH device needs to be reestablished */
    /* Should state be put into the errors or the error system within CarosNodeServiceInterface? (It is not guaranteed that a locally tracked error state is not being superseded by another error cause somewhere else in the CAROS system - so such a solution would be prone to errors) */

    /* Remember to place the state machine in a proper state according to the recovery (e.g. WAIT) */
    
    ROS_ERROR_STREAM("The recoverHook() has not been implemented yet!");

    return false;
}

void PG70Node::initLoopHook() {
/* Empty */
}

void PG70Node::stoppedLoopHook() {
/* Empty */
}

void PG70Node::runLoopHook() {
    try {
        if (_pg70 == 0) {
            CAROS_FATALERROR("The PG70 device is not configured", PG70NODE_INTERNAL_ERROR);
            return;
        }

        if (! _pg70->isConnected()) {
            CAROS_ERROR("There is no established connection to the PG70 device.", PG70NODE_PG70_DEVICE_NO_CONNECTION);
            return;
        }

        /************************************************************************
         * Get Current Joint Positions
         * Currently used as part of the workaround for the movement of the SDH fingers.
        ************************************************************************/
        _pg70->getQ(_currentQ);

        /************************************************************************
         * Velocity Calculation
         * Used as part of the workaround for the movement of the SDH fingers.
        ************************************************************************/
        double time = _velUpdateTimer.getTime();
        
        if (_lastQ.size() != _currentQ.size()) {
            /* _lastq has not been set before (first time the runlookHook is being called), so set it to _currentQ */
            _lastQ = _currentQ;
        } else if (time > MIN_TIME_THRESHOLD_FOR_CALCULATING_VELOCITY) {
            /* calculate velocity */
            _velQ = (_lastQ-_currentQ)/time;
            _lastQ = _currentQ;
            _velUpdateTimer.resetAndResume();
            ROS_DEBUG_STREAM_NAMED("velocity", "New calculated velocity: " << _velQ);
        }
        ROS_DEBUG_STREAM_NAMED("velocity", "Calculated velocity: " << _velQ);

        /************************************************************************
         * Publish SDH state
         ************************************************************************/
        /* Publishing the SDH state before the state machine because then the measured/sampled values will (probably) have the best match with the current SDH action(s). Reporting a speed and having isstop being true is sort of inconsistent. */
        /* The units of the reported values should match what is specified in the GripperState.msg specification.
         * The rwhw::SDHDriver constructor specifies the use of radians.
         */
        rw::math::Q q = _currentQ;
        /* Using the calculated velocity (FIXME: either continue to use _velQ or use _sdh->getdQ() if they report similar values - see the velocity debug messages) */
        rw::math::Q dq = _velQ;
        /* FIXME: the current could be converted to a force, given it would make sense to do so - but it requires knowledge of the kinematics to calculate the force that is being applied e.g. at a particular fingertip or where the contact surface is. */
        rw::math::Q force = Q(0);//_pg70->getQ();

        rw::math::Q compare = rw::math::Q::zero(dq.size());
        bool isMoving = (compare != dq) ? true : false;
        bool isBlocked = false;
        /* FIXME: This can possibly give a wrong report in the situation where a new moveQ has just been initiated - so the calculated distance between current position and target is greater than the threshold, but the measured/calculated velocity is still zero */
        if (!isMoving && (rw::math::MetricUtil::dist2(_currentQ, _moveQ) >= MOVE_DISTANCE_STOPPED_THRESHOLD)) {
            isBlocked = true;
        }
        bool isStopped = true;
        /* If not moving nor blocked, then it must be stopped (i.e. reached the target (see GripperState.msg specification)) */
        if (isMoving || isBlocked) {
            isStopped = false;
        }
        /* TODO: properly handle isEmergencyStopped - the logic to register whether an emergency stop is activated or deactivated is missing. Returning false or true in this situation is not recommended though... */
        bool isEmergencyStopped = false;
        publishState(q,dq,force,isMoving,isBlocked,isStopped,isEmergencyStopped);

        /************************************************************************
         * State Machine
         * Used to apply workaround for the movement of the SDH fingers.
        ************************************************************************/
 /*       _currentState = _nextState;
        switch (_currentState) {
        case WAIT:
            /* Do nothing */
 /*           break;
        case MOVE_WAIT:
            /* Workaround to avoid having the SDH try to move the fingers until MAX_TIME_WAITING_FOR_MOVE_TO_FINISH_BEFORE_INTERVENING has passed, when the fingers are almost at their target - (there should be a "bug" causing the SDH to dissipate power when the target can't be reached according to the firmware) */
/*            if (rw::math::MetricUtil::dist2(_currentQ, _moveQ) < MOVE_DISTANCE_STOPPED_THRESHOLD) {
                /* Debug functionality to test the usage of _sdh->waitCmd(0) instead of looking at the remaining distance to the target */
/*                ROS_DEBUG_STREAM_NAMED("move_wait", "_sdh->waitCmd(0) returned: " << _sdh->waitCmd(0));
                
                _sdh->stop();
                _nextState = WAIT;
            } else if (_moveStartTimer.getTime() > MAX_TIME_WAITING_FOR_MOVE_TO_FINISH_BEFORE_INTERVENING) {
                ROS_DEBUG_STREAM_NAMED("move_wait", "Waited long enough to possible intervene.");
                if (rw::math::MetricUtil::normInf(_velQ) < MINIMUM_VELOCITY_BEFORE_CONSIDERED_NOT_MOVING) {
                    ROS_DEBUG_STREAM_NAMED("move_wait", "Intervening due to the fingers not being considered moving.");
                    _sdh->stop();
                    _nextState = WAIT;
                }
            }
            break;
        default:
            ROS_FATAL_STREAM("Unknown state in the SDH state machine '" << _currentState << "' (a value is expected due to enum implementation) - This is a bug!");
            /* This is considered a fatal error, but should never happen. */
 /*           CAROS_FATALERROR("Unknown state in the SDH state machine", SDHNODE_INTERNAL_ERROR);
            break;
        }
    
 */
    } catch(const rw::common::Exception& exp) {
        /* TODO: Digest the exp.what() string and find more appropriate error codes - or improve rwhw::SDHDriver to throw more c++ standard exceptions with out-of-range and invalid parameter specific exceptions. Thus making it easier to do error recovery. */
       CAROS_ERROR(exp.what(), PG70NODE_INTERNAL_ERROR);
        return;
    }
   
}

void PG70Node::errorLoopHook() {
    /* Stop the SDH's current action(s) */
    if (_pg70 == 0) {
        ROS_DEBUG_STREAM("The PG70 device was not configured when '" << __PRETTY_FUNCTION__ << "' was invoked!");
    } else {
        if (_pg70->isConnected()) {
            _pg70->stop();
        }
    }
}

void PG70Node::fatalErrorLoopHook() {
    /* Stop the SDH's current action(s) */
    if (_pg70 == 0) {
        ROS_DEBUG_STREAM("The PG70 device was not configured when '" << __PRETTY_FUNCTION__ << "' was invoked!");
    } else {
        if (_pg70->isConnected()) {
            _pg70->stop();
            /* A fatal error should disconnect the SDH device */
            _pg70->disconnect();
        }
    }
}

bool PG70Node::gripQ(const rw::math::Q& q)
{
    ROS_DEBUG_STREAM_NAMED("received_q", "gripQ: " << q);

    if (!isInRunning()) {
        ROS_WARN_STREAM("Not in running state!");
        return false;
    }

    if (_pg70 == 0) {
        CAROS_FATALERROR("The PG70 device is not configured.", PG70NODE_NO_PG70_DEVICE);
        return false;
    }

    if (! _pg70->isConnected()) {
        CAROS_ERROR("There is no established connection to the PG70 device.", PG70NODE_PG70_DEVICE_NO_CONNECTION);
        return false;
    }

    if (q.size() != SUPPORTED_Q_LENGTH_FOR_PG70NODE) {
        CAROS_ERROR("The length of Q is " << q.size() << " but should be " << SUPPORTED_Q_LENGTH_FOR_PG70NODE, PG70NODE_UNSUPPORTED_Q_LENGTH);
        return false;
    }

    try {
        _pg70->setQ(q);
        /* Do nothing; letting the SDH continue to apply force as part of its grasp */
        //_nextState = WAIT;
    } catch (const rw::common::Exception& exp) {
        /* TODO: Digest the exp.what() string and find more appropriate error codes - or improve rwhw::SDHDriver to throw more c++ standard exceptions with out-of-range and invalid parameter specific exceptions. Thus making it easier to do error recovery. */
        CAROS_ERROR(exp.what(), PG70NODE_INTERNAL_ERROR);
        return false;
    }

    return true;
}

bool PG70Node::setForceQ(const rw::math::Q& q)
{
    ROS_ERROR("caros_pg70_node: setForceQ not implemented!!");
    return false;
}

bool PG70Node::moveQ(const rw::math::Q& q)
{
  ROS_ERROR("caros_pg70_node: moveQ not implemented!!");
  return false;
}

bool PG70Node::setVelocityQ(const rw::math::Q& q)
{
    ROS_DEBUG_STREAM_NAMED("received_q", "setVelocityQ: " << q);

    if (!isInRunning()) {
        ROS_WARN_STREAM("Not in running state!");
        return false;
    }

    if (_pg70 == 0) {
        CAROS_FATALERROR("The PG70 device is not configured.", PG70NODE_NO_PG70_DEVICE);
        return false;
    }

    if (! _pg70->isConnected()) {
        CAROS_ERROR("There is no established connection to the SDH device.", PG70NODE_PG70_DEVICE_NO_CONNECTION);
        return false;
    }

    if (q.size() != SUPPORTED_Q_LENGTH_FOR_PG70NODE) {
        CAROS_ERROR("The length of Q is " << q.size() << " but should be " << SUPPORTED_Q_LENGTH_FOR_PG70NODE, PG70NODE_UNSUPPORTED_Q_LENGTH);
        return false;
    }

    try {
      //_pg70->setTargetQVel(q);
      if(q.size() > 1){
	 CAROS_ERROR("The length of Q is " << q.size() << " but should be " << 1 << "when setting the velocity. Q has to be from 0-100 (GraspPowerPct)", PG70NODE_UNSUPPORTED_Q_LENGTH);
	 return false;
      }else if(q(0) < 0 || q(0) > 100){
	 CAROS_ERROR("Setting Velocity requires a Q(0) between 0-100(PG70 specific cmd: setGraspPowerPct)", PG70NODE_UNSUPPORTED_Q_LENGTH);
	 return false;
      }
      
      _pg70->setGraspPowerPct(q(0));
      
    } catch (const rw::common::Exception& exp) {
        /* TODO: Digest the exp.what() string and find more appropriate error codes - or improve rwhw::SDHDriver to throw more c++ standard exceptions with out-of-range and invalid parameter specific exceptions. Thus making it easier to do error recovery. */
        CAROS_ERROR(exp.what(), PG70NODE_INTERNAL_ERROR);
        return false;
    }

    return true;
}

bool PG70Node::stopMovement()
{
    if (!isInRunning()) {
        ROS_WARN_STREAM("Not in running state!");
        return false;
    }

    if (_pg70 == 0) {
        CAROS_FATALERROR("The PG70 device is not configured.", PG70NODE_NO_PG70_DEVICE);
        return false;
    }

    if (! _pg70->isConnected()) {
        CAROS_ERROR("There is no established connection to the SDH device.", PG70NODE_PG70_DEVICE_NO_CONNECTION);
        return false;
    }

    try {
        _pg70->stop();
       // _nextState = WAIT;
    } catch (const rw::common::Exception& exp) {
        /* TODO: Digest the exp.what() string and find more appropriate error codes - or improve rwhw::SDHDriver to throw more c++ standard exceptions with out-of-range and invalid parameter specific exceptions. Thus making it easier to do error recovery. */
        CAROS_ERROR(exp.what(), PG70NODE_INTERNAL_ERROR);
        return false;
    }

    return true;
}


bool PG70Node::home()
{
	_pg70->home();
	return true;
}


bool PG70Node::open(float force)
{
	std::cout<<"Open "<<force<<std::endl;
	_pg70->setGraspPowerPct(force);
	_pg70->open();
	return true;
}

bool PG70Node::close(float force)
{
	std::cout<<"Close "<<force<<std::endl;
	_pg70->setGraspPowerPct(force);
	_pg70->close();
	return true;
}
