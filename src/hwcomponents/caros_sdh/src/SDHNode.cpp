#include <caros/SDHNode.hpp>

#include <caros/CarosNodeServiceInterface.hpp>
#include <caros/GripperServiceInterface.hpp>

#include <rw/math/Q.hpp>
#include <rw/common/Timer.hpp>
#include <rw/common/Exception.hpp>
#include <rw/math/MetricUtil.hpp>

#include <rwhw/sdh/SDHDriver.hpp>

#include <ros/ros.h>

#include <string>
#include <sstream>
#include <utility>
#include <cstddef> // Provides NULL

/* Notes:
 * This node is designed to be run in a single thread that doesn't allow concurrently processing of the set-commands and/or the runLoopHook - eliminating the possibility of race conditions.
 * The GripperServiceInterface commands (and other services that are configured/advertised in the configureHook()) can be called even when the CarosNode is in a non-running state. This is due to the design choice that all the services and publishers should be broadcasted/available once the CarosNode has been configured. This will avoid hiding (some of) the capabilities/interface of the node when it's not in certain states. However this requires more defensive programming on the interface methods, to make sure that they will only work when the CarosNode has been put in the running state (and connections to the hardware/device has been established or similar).
 */

/* TODO:
 * The node implementation can be greatly simplified if the workaround for the movement of the SDH fingers can be placed in RobWorkHardware or simply removed (requires collecting the debug data and comparing with the SDH temperature).
 * Add support for emergency stop. This can be triggered either through CarosNodeServiceInterface or directly to this node. The SDH library contains a function for emergency stop, which should be used and called immediately.
 *   - Also handle emergency stop properly in the SDHGripperServiceInterface.
 *   - What is required to actually release the emergencystop (within the rwhw::SDHDriver / SDHLibrary)?
 *
 * Allow specification of what interface to use/configure through the parameter server.
 *
 * Could keep track of whether the services/commands (moveQ, gripQ, etc.) finish executing before new commands are received (it's not an error, but maybe it would be nice to get some debug/info information regarding this. Hopefully it would make it easier to reason/debug "what just happened").
 */

SDHNode::SDHNode(const ros::NodeHandle& nodehandle):
    caros::CarosNodeServiceInterface(nodehandle),
    caros::GripperServiceInterface(nodehandle),
    _nodeHandle(nodehandle),
    _currentState(SDHNode::WAIT),
    _nextState(_currentState),
    _sdh(0)
{
    /* Currently nothing specific should happen */
}

SDHNode::~SDHNode() {
    if (_sdh != 0) {
        if (_sdh->isConnected()) {
            ROS_DEBUG_STREAM("Still connected to the SDH device - going to stop the device and disconnect.");
            _sdh->stop();
            _sdh->disconnect();
        }
        delete _sdh;
        _sdh = 0;
    }
}

bool SDHNode::configureHook() {
    if (_sdh != 0) {
        CAROS_FATALERROR("The SDH device is already active - please properly cleanup before trying to configure the SDH.", SDHNODE_SDH_DEVICE_ALREADY_ACTIVE);
        return false;
    }
    _sdh = new rwhw::SDHDriver;

    /* Fetch parameters (if any) or use the defaults */
    _nodeHandle.param("interface_type", _interfaceType, std::string("CAN"));

    _nodeHandle.param("rs232_device", _rs232Device, std::string(""));
    _nodeHandle.param("rs232_port", _rs232Port, 0);
    _nodeHandle.param("rs232_baudrate", _rs232BaudRate, 115200);
    _nodeHandle.param("rs232_timeout", _rs232Timeout, 0.5);

    _nodeHandle.param("can_device", _canDevice, std::string("/dev/pcan0"));
    _nodeHandle.param("can_baudrate", _canBaudRate, 1000000);
    _nodeHandle.param("can_timeout", _canTimeout, 0.5);

    /* TODO: Verify that the chosen interfaceType is valid? or just let it fail when start is invoked? */

    /* TODO: Could make the use of the gripper service, configurable through the parameter server. */
    if (!configureGripperService()) {
        CAROS_FATALERROR("The CAROS GripperService could not be configured correctly.", SDHNODE_CAROS_GRIPPER_SERVICE_CONFIGURE_FAIL);
        return false;
    }

    /* Outputting information on supported value ranges */
    /* TODO: This could be made part of the GripperServiceInterface - possibly as a message that is returned (or published) when a client asks for it.
     * If the hardware is intelligent enough to provide new values/boundaries according to position or grasping mode, then it could make sense to publish that information when it changes
     */
    std::pair<rw::math::Q, rw::math::Q> positionLimits = _sdh->getPosLimits();
    rw::math::Q velocityLimits = _sdh->getVelLimits();
    /* There's also getAccLimits() */
    rw::math::Q currentLimits = _sdh->getCurrentLimits();

    ROS_ERROR_STREAM_COND(positionLimits.first.size() != positionLimits.second.size(), "The sizes of the Q's in the position limit pair are not equal; first contains " << positionLimits.first.size() << " and second contains " << positionLimits.second.size() << " elements.");

    ROS_INFO_STREAM("Lower position limits: " << positionLimits.first);
    ROS_INFO_STREAM("Upper position limits: " << positionLimits.second);
    ROS_INFO_STREAM("Velocity limits: " << velocityLimits);
    ROS_INFO_STREAM("Current limits: " << currentLimits);

    /* TODO: Debug information on what was configured accoringly to the parameter server? */
    return true;
}

bool SDHNode::cleanupHook() {
    /* Keep cleaning up even if the gripper service can't be cleaned up properly */
    bool retVal = true;
    if (!cleanupGripperService()) {
        retVal = false;
        ROS_ERROR_STREAM("cleanupGripperService() failed.");
    }

    ROS_WARN_STREAM_COND(_sdh == 0, "cleanupHook() was called with the SDH device not being configured (i.e. no valid SDH-object)"); 
    if (_sdh != 0) {
        if (_sdh->isConnected()) {
            ROS_DEBUG_STREAM("Still connected to the SDH device - going to stop the device and disconnect.");
            _sdh->stop();
            _sdh->disconnect();
        }
        delete _sdh;
        _sdh = 0;
    }

    return retVal;
}

bool SDHNode::startHook() {
    if (_sdh == 0) {
        CAROS_FATALERROR("The SDH device is not configured", SDHNODE_INTERNAL_ERROR);
        return false;
    }

    if (_sdh->isConnected()) {
        /* Not jumping to an error state, thus allowing further use after having startHook() mistakenly invoked.
         * This can happen for a variety of reasons, of which some are:
         * - A "misconfiguration" - some other node just invoking start on this node without verifying that this node is not currently running
         * - A race condition (multiple nodes trying to start this node - although highly unlikely when the callback queues are processed one by one)
         */
        ROS_ERROR_STREAM("'" << __PRETTY_FUNCTION__ << "' invoked even though a connection to the SDH device has already been established - this should be considered a bug!"); 
        return false;
    }

    /* Connect according to interface type and configured parameters */
    if (_interfaceType == "RS232") {
        if (_rs232Device.empty()) {
            _sdh->connect(_rs232Port, static_cast<unsigned long>(_rs232BaudRate), _rs232Timeout, NULL);
        } else {
            _sdh->connect(_rs232Port, static_cast<unsigned long>(_rs232BaudRate), _rs232Timeout, _rs232Device.c_str());
        }
    } else if (_interfaceType == "CAN") {
        _sdh->connect(_canDevice, _canBaudRate, _canTimeout);
    } else {
        CAROS_FATALERROR("The specified interface '" << _interfaceType << "' is not supported.", SDHNODE_UNSUPPORTED_INTERFACE_TYPE);
        return false;
    }

    /* Verify that the connection to the SDH has been established - this eliminates the need for verifying the _sdh->connect() function calls actually succeeded */
    if (! _sdh->isConnected()) {
        /* Something went wrong when connecting */
        CAROS_FATALERROR("Failed to properly connect to the SDH device.", SDHNODE_SDH_DEVICE_CONNECT_FAILED);
        return false;
    }

    return true;
}

bool SDHNode::stopHook() {
    if (_sdh == 0) {
        CAROS_FATALERROR("The SDH device is not configured", SDHNODE_INTERNAL_ERROR);
        return false;
    } else {
        if (_sdh->isConnected()) {
            /* TODO: calling stop() is probably unnecessary */
            _sdh->stop();
            _sdh->disconnect();
        } else {
            ROS_WARN_STREAM("There was no established connection to the SDH device when '" << __PRETTY_FUNCTION__ << "' was invoked! - Consider this a bug!");
        }
    }

    return true;
}

bool SDHNode::recoverHook() {
/* TODO: */
    /* Maybe the connection to the SDH device needs to be reestablished */
    /* Should state be put into the errors or the error system within CarosNodeServiceInterface? (It is not guaranteed that a locally tracked error state is not being superseded by another error cause somewhere else in the CAROS system - so such a solution would be prone to errors) */

    /* Remember to place the state machine in a proper state according to the recovery (e.g. WAIT) */
    
    ROS_ERROR_STREAM("The recoverHook() has not been implemented yet!");

    return false;
}

void SDHNode::initLoopHook() {
/* Empty */
}

void SDHNode::stoppedLoopHook() {
/* Empty */
}

void SDHNode::runLoopHook() {
    try {
        if (_sdh == 0) {
            CAROS_FATALERROR("The SDH device is not configured", SDHNODE_INTERNAL_ERROR);
            return;
        }

        if (! _sdh->isConnected()) {
            CAROS_ERROR("There is no established connection to the SDH device.", SDHNODE_SDH_DEVICE_NO_CONNECTION);
            return;
        }

        /************************************************************************
         * Get Current Joint Positions
         * Currently used as part of the workaround for the movement of the SDH fingers.
        ************************************************************************/
        _currentQ = _sdh->getQ();

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
        ROS_DEBUG_STREAM_NAMED("velocity", "SDH reported velocity: " << _sdh->getdQ());

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
        rw::math::Q force = _sdh->getQCurrent();

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
        _currentState = _nextState;
        switch (_currentState) {
        case WAIT:
            /* Do nothing */
            break;
        case MOVE_WAIT:
            /* Workaround to avoid having the SDH try to move the fingers until MAX_TIME_WAITING_FOR_MOVE_TO_FINISH_BEFORE_INTERVENING has passed, when the fingers are almost at their target - (there should be a "bug" causing the SDH to dissipate power when the target can't be reached according to the firmware) */
            if (rw::math::MetricUtil::dist2(_currentQ, _moveQ) < MOVE_DISTANCE_STOPPED_THRESHOLD) {
                /* Debug functionality to test the usage of _sdh->waitCmd(0) instead of looking at the remaining distance to the target */
                ROS_DEBUG_STREAM_NAMED("move_wait", "_sdh->waitCmd(0) returned: " << _sdh->waitCmd(0));
                
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
            CAROS_FATALERROR("Unknown state in the SDH state machine", SDHNODE_INTERNAL_ERROR);
            break;
        }
    } catch(const rw::common::Exception& exp) {
        /* TODO: Digest the exp.what() string and find more appropriate error codes - or improve rwhw::SDHDriver to throw more c++ standard exceptions with out-of-range and invalid parameter specific exceptions. Thus making it easier to do error recovery. */
        CAROS_ERROR(exp.what(), SDHNODE_INTERNAL_ERROR);
        return;
    }
}

void SDHNode::errorLoopHook() {
    /* Stop the SDH's current action(s) */
    if (_sdh == 0) {
        ROS_DEBUG_STREAM("The SDH device was not configured when '" << __PRETTY_FUNCTION__ << "' was invoked!");
    } else {
        if (_sdh->isConnected()) {
            _sdh->stop();
        }
    }
}

void SDHNode::fatalErrorLoopHook() {
    /* Stop the SDH's current action(s) */
    if (_sdh == 0) {
        ROS_DEBUG_STREAM("The SDH device was not configured when '" << __PRETTY_FUNCTION__ << "' was invoked!");
    } else {
        if (_sdh->isConnected()) {
            _sdh->stop();
            /* A fatal error should disconnect the SDH device */
            _sdh->disconnect();
        }
    }
}

/************************************************************************
 * GripperServiceInterface
 ************************************************************************/
bool SDHNode::moveQ(const rw::math::Q& q)
{
    ROS_DEBUG_STREAM_NAMED("received_q", "moveQ: " << q);

    if (!isInRunning()) {
        ROS_WARN_STREAM("Not in running state!");
        return false;
    }

    if (_sdh == 0) {
        CAROS_FATALERROR("The SDH device is not configured.", SDHNODE_NO_SDH_DEVICE);
        return false;
    }

    if (! _sdh->isConnected()) {
        CAROS_ERROR("There is no established connection to the SDH device.", SDHNODE_SDH_DEVICE_NO_CONNECTION);
        return false;
    }

    if (q.size() != SUPPORTED_Q_LENGTH_FOR_SDHNODE) {
        CAROS_ERROR("The length of Q is " << q.size() << " but should be " << SUPPORTED_Q_LENGTH_FOR_SDHNODE, SDHNODE_UNSUPPORTED_Q_LENGTH);
        return false;
    }

    try {
        _moveQ = q;
        _moveStartTimer.resetAndResume();

        _sdh->moveCmd(_moveQ);
        _nextState = MOVE_WAIT;
    } catch (const rw::common::Exception& exp) {
        /* TODO: Digest the exp.what() string and find more appropriate error codes - or improve rwhw::SDHDriver to throw more c++ standard exceptions with out-of-range and invalid parameter specific exceptions. Thus making it easier to do error recovery. */
        CAROS_ERROR(exp.what(), SDHNODE_INTERNAL_ERROR);
        return false;
    }

    return true;
}

bool SDHNode::gripQ(const rw::math::Q& q)
{
    ROS_DEBUG_STREAM_NAMED("received_q", "gripQ: " << q);

    if (!isInRunning()) {
        ROS_WARN_STREAM("Not in running state!");
        return false;
    }

    if (_sdh == 0) {
        CAROS_FATALERROR("The SDH device is not configured.", SDHNODE_NO_SDH_DEVICE);
        return false;
    }

    if (! _sdh->isConnected()) {
        CAROS_ERROR("There is no established connection to the SDH device.", SDHNODE_SDH_DEVICE_NO_CONNECTION);
        return false;
    }

    if (q.size() != SUPPORTED_Q_LENGTH_FOR_SDHNODE) {
        CAROS_ERROR("The length of Q is " << q.size() << " but should be " << SUPPORTED_Q_LENGTH_FOR_SDHNODE, SDHNODE_UNSUPPORTED_Q_LENGTH);
        return false;
    }

    try {
        _sdh->moveCmd(q);
        /* Do nothing; letting the SDH continue to apply force as part of its grasp */
        _nextState = WAIT;
    } catch (const rw::common::Exception& exp) {
        /* TODO: Digest the exp.what() string and find more appropriate error codes - or improve rwhw::SDHDriver to throw more c++ standard exceptions with out-of-range and invalid parameter specific exceptions. Thus making it easier to do error recovery. */
        CAROS_ERROR(exp.what(), SDHNODE_INTERNAL_ERROR);
        return false;
    }

    return true;
}


bool SDHNode::setForceQ(const rw::math::Q& q)
{
    ROS_DEBUG_STREAM_NAMED("received_q", "setForceQ: " << q);

    if (!isInRunning()) {
        ROS_WARN_STREAM("Not in running state!");
        return false;
    }

    if (_sdh == 0) {
        CAROS_FATALERROR("The SDH device is not configured.", SDHNODE_NO_SDH_DEVICE);
        return false;
    }

    if (! _sdh->isConnected()) {
        CAROS_ERROR("There is no established connection to the SDH device.", SDHNODE_SDH_DEVICE_NO_CONNECTION);
        return false;
    }

    if (q.size() != SUPPORTED_Q_LENGTH_FOR_SDHNODE) {
        CAROS_ERROR("The length of Q is " << q.size() << " but should be " << SUPPORTED_Q_LENGTH_FOR_SDHNODE, SDHNODE_UNSUPPORTED_Q_LENGTH);
        return false;
    }

    try {
        _sdh->setTargetQCurrent(q);
    } catch (const rw::common::Exception& exp) {
        /* TODO: Digest the exp.what() string and find more appropriate error codes - or improve rwhw::SDHDriver to throw more c++ standard exceptions with out-of-range and invalid parameter specific exceptions. Thus making it easier to do error recovery. */
        CAROS_ERROR(exp.what(), SDHNODE_INTERNAL_ERROR);
        return false;
    }

    return true;
}

bool SDHNode::setVelocityQ(const rw::math::Q& q)
{
    ROS_DEBUG_STREAM_NAMED("received_q", "setVelocityQ: " << q);

    if (!isInRunning()) {
        ROS_WARN_STREAM("Not in running state!");
        return false;
    }

    if (_sdh == 0) {
        CAROS_FATALERROR("The SDH device is not configured.", SDHNODE_NO_SDH_DEVICE);
        return false;
    }

    if (! _sdh->isConnected()) {
        CAROS_ERROR("There is no established connection to the SDH device.", SDHNODE_SDH_DEVICE_NO_CONNECTION);
        return false;
    }

    if (q.size() != SUPPORTED_Q_LENGTH_FOR_SDHNODE) {
        CAROS_ERROR("The length of Q is " << q.size() << " but should be " << SUPPORTED_Q_LENGTH_FOR_SDHNODE, SDHNODE_UNSUPPORTED_Q_LENGTH);
        return false;
    }

    try {
        _sdh->setTargetQVel(q);
    } catch (const rw::common::Exception& exp) {
        /* TODO: Digest the exp.what() string and find more appropriate error codes - or improve rwhw::SDHDriver to throw more c++ standard exceptions with out-of-range and invalid parameter specific exceptions. Thus making it easier to do error recovery. */
        CAROS_ERROR(exp.what(), SDHNODE_INTERNAL_ERROR);
        return false;
    }

    return true;
}

bool SDHNode::stopMovement()
{
    if (!isInRunning()) {
        ROS_WARN_STREAM("Not in running state!");
        return false;
    }

    if (_sdh == 0) {
        CAROS_FATALERROR("The SDH device is not configured.", SDHNODE_NO_SDH_DEVICE);
        return false;
    }

    if (! _sdh->isConnected()) {
        CAROS_ERROR("There is no established connection to the SDH device.", SDHNODE_SDH_DEVICE_NO_CONNECTION);
        return false;
    }

    try {
        _sdh->stop();
        _nextState = WAIT;
    } catch (const rw::common::Exception& exp) {
        /* TODO: Digest the exp.what() string and find more appropriate error codes - or improve rwhw::SDHDriver to throw more c++ standard exceptions with out-of-range and invalid parameter specific exceptions. Thus making it easier to do error recovery. */
        CAROS_ERROR(exp.what(), SDHNODE_INTERNAL_ERROR);
        return false;
    }

    return true;
}
