#include <caros/CarosNodeServiceInterface.hpp>
#include <caros_common_msgs/CarosNodeState.h>

#include <ros/ros.h>

#include <string>

/************************************************************************
 * TODO:
 * - Should the publish rate on the NodeState be lowered to maybe 1 or
 *   2 Hz when in an error state?
 * - Make it possible to configure the _loopRate through the parameter
 *   server.
 ************************************************************************/

namespace caros {

    namespace {
        static std::string CarosStateString[] = {"PREINIT","STOPPED","RUNNING","INERROR","INFATALERROR"};
    }

    CarosNodeServiceInterface::CarosNodeServiceInterface(const ros::NodeHandle& nodehandle, const double loopRateFrequency):
        _nodeHandle(nodehandle, CAROS_NODE_SERVICE_INTERFACE_SUB_NAMESPACE),
        _nodeState(PREINIT),
        _loopRateFrequency(loopRateFrequency),
        _loopRate(_loopRateFrequency)
    {
        if (initCarosNode()) {
            /* TODO:
             * FIXME:
             *   How to react on an unsuccessful initialisation? - zombie object or throw an exception
             */
        }
    }

    CarosNodeServiceInterface::~CarosNodeServiceInterface() {
        /* Nothing to destroy */
    }

    void CarosNodeServiceInterface::start() {
        caros_common_msgs::CarosNodeState state;
        while (ros::ok()) {
            ros::spinOnce();

            if (_nodeState == RUNNING) {
                runLoopHook();
            } else if (_nodeState == STOPPED) {
                stoppedLoopHook();
            } else if (_nodeState == PREINIT) {
                initLoopHook();
            }

            /* Process errors if any occurred */
            if (_nodeState == INERROR) {
                errorLoopHook();
            }
            /* Also process fatal error state if an error should become a fatal error */
            if (_nodeState == INFATALERROR) {
                fatalErrorLoopHook();
            }

            publishNodeState();

            /* Sleep to run approximately at the specified Hz */
            _loopRate.sleep();
            /* TODO:
             * Add debug statements regarding the return value of the sleep() - it should return true if the desired rate was met for this cycle, else false
             */
        }
    }

    /* TODO: These methods can just be converted to void, as the return value will always be true (except for cleanupNode()) */
    bool CarosNodeServiceInterface::configureNode() {
        if (_nodeState != PREINIT) {
            ROS_ERROR_STREAM("Configure can only be called from the " << CarosStateString[PREINIT] << " state. The node was in " << CarosStateString[_nodeState] << ".");
            return false;
        }

        if (configureHook()) {
            changeState(STOPPED);
        } else {
            ROS_DEBUG_STREAM("configureHook() failed.");
            /* TODO:
             * Should verify that the node has been put into an (fatal)error state. If that didn't happen then invoke fatalError() and possibly output a warning stating that the node/user should implement proper error handling code before returning false from the cleanupHook()
             */
            return false;
        }

        return true;
    }

    bool CarosNodeServiceInterface::cleanupNode() {
        if (_nodeState == PREINIT) {
            ROS_WARN_STREAM("It does not make much sense to invoke cleanupNode() when in the " << CarosStateString[_nodeState] << " state!");
            return true;
        }

        if (_nodeState == RUNNING) {
            ROS_DEBUG_STREAM("Node is in running state - stopping node.");
            if (!stopNode()) {
                ROS_ERROR_STREAM("Was not able to properly stop the node.");
                fatalError("Something went wrong while stopping the node within the cleanup step.");
                return false;
            }
        }

        if (cleanupHook()) {
            changeState(PREINIT);
        } else {
            ROS_DEBUG_STREAM("cleanupHook() failed.");
            /* TODO:
             * Should verify that the node has been put into an (fatal)error state. If that didn't happen then invoke fatalError() and possibly output a warning stating that the node/user should implement proper error handling code before returning false from the cleanupHook()
             */
            return false;
        }

        return true;
    }

    bool CarosNodeServiceInterface::startNode() {
        // can only be called in stopped
        if (_nodeState != STOPPED) {
            ROS_ERROR_STREAM("Start can only be called when in " << CarosStateString[STOPPED] << " state. The node was in " << CarosStateString[_nodeState] << ".");
            return false;
        }

        if (startHook()) {
            changeState(RUNNING);
        } else {
            ROS_DEBUG_STREAM("startHook() failed.");
            /* TODO:
             * Should verify that the node has been put into an (fatal)error state. If that didn't happen then invoke error() and possibly output a warning stating that the node/user should implement proper error handling code before returning false from the startHook()
             */
            return false;
        }

        return true;
    }

    bool CarosNodeServiceInterface::stopNode() {
        if (_nodeState != RUNNING && _nodeState != INERROR ) {
            ROS_ERROR_STREAM("Stop can only be called from the " << CarosStateString[RUNNING] << " or " << CarosStateString[INERROR] << " states. The node was in " << CarosStateString[_nodeState] << ".");
            return false;
        }

        if (stopHook()) {
            changeState(STOPPED);
        } else {
            ROS_DEBUG_STREAM("stopHook() failed.");
            /* TODO:
             * Should verify that the node has been put into an (fatal)error state. If that didn't happen then invoke (fatal)error() and possibly output a warning stating that the node/user should implement proper error handling code before returning false from the stopHook()
             */
            return false;
        }

        return true;
    }

    bool CarosNodeServiceInterface::recoverNode() {
        // can only be called when in error state
        if(_nodeState != INERROR) {
            ROS_WARN_STREAM("Recover can only be called from " << CarosStateString[INERROR] << " state. The node was in " << CarosStateString[_nodeState] << ".");
            return false;
        }
        
        if (recoverHook()) {
            ROS_ERROR_STREAM_COND(_previousState == INFATALERROR, "A successful recovery brings the node back into the " << CarosStateString[INFATALERROR] << " state - This is a bug!");
            changeState(_previousState);
        } else {
            ROS_DEBUG_STREAM("recoverHook() failed.");
            /* TODO:
             * What to do if the recovery fails? Go into fatalError? or is that up to the node implementer to entirely decide?
             * Should verify that the node has been put into an (fatal)error state. If that didn't happen then invoke (fatal)error() and possibly output a warning stating that the node/user should implement proper error handling code before returning false from the recoverHook()
             */
            return false;
        }

        return true;
    }

    void CarosNodeServiceInterface::error(const std::string& msg, const int64_t errorCode) {
        ROS_DEBUG_STREAM("CarosNodeError: " << msg << "; error code: " << errorCode);
        /* keep a copy of the error message so it can be published */
        _errorMsg = msg;
        _errorCode = errorCode;
        changeState(INERROR);
    }

    void CarosNodeServiceInterface::fatalError(const std::string& msg, const int64_t errorCode) {
        ROS_DEBUG_STREAM("CarosNodeFatalError: " << msg << "; error code: " << errorCode);
        /* keep a copy of the (fatal) error message so it can be published */
        _errorMsg = msg;
        _errorCode = errorCode;
        changeState(INFATALERROR);
    }

    void CarosNodeServiceInterface::setLoopRateFrequency(const double frequency) {
        ROS_DEBUG_STREAM("Changing the loop rate frequency from " << _loopRateFrequency << " to " << frequency);
        _loopRateFrequency = frequency;
        _loopRate = ros::Rate(_loopRateFrequency);
    }

    void CarosNodeServiceInterface::changeState(const NodeState newState) {
        ROS_DEBUG_STREAM("Changing state from " << CarosStateString[_nodeState] << " to " << CarosStateString[newState]);
        if (newState != _nodeState) {
            _previousState = _nodeState;
            _nodeState = newState;
            publishNodeState(true);
        } else {
            ROS_DEBUG_STREAM("Not changing state as the new state is the same as the current state!");
        }
    }

    bool CarosNodeServiceInterface::initCarosNode(){
        if (_nodeStatePublisher || _srvStop || _srvStart || _srvConfigure || _srvCleanup || _srvRecover) {
            ROS_WARN_STREAM("Reinitialising one or more CarosNodeServiceInterface services or publishers. If this is not fully intended then this should be considered a bug!");
        }

        _nodeStatePublisher = _nodeHandle.advertise<caros_common_msgs::CarosNodeState>("CarosNodeState", 1);
        ROS_ERROR_STREAM_COND(!_nodeStatePublisher, "The CarosNodeState publisher is empty!");

        _srvStop  = _nodeHandle.advertiseService("stop", &CarosNodeServiceInterface::stopHandle, this);
        ROS_ERROR_STREAM_COND(!_srvStop , "The stop service is empty!");

        _srvStart = _nodeHandle.advertiseService("start", &CarosNodeServiceInterface::startHandle, this);
        ROS_ERROR_STREAM_COND(!_srvStart, "The start service is empty!");

        _srvConfigure = _nodeHandle.advertiseService("configure", &CarosNodeServiceInterface::configureHandle, this);
        ROS_ERROR_STREAM_COND(!_srvConfigure, "The configure service is empty!");

        _srvCleanup = _nodeHandle.advertiseService("cleanup", &CarosNodeServiceInterface::cleanupHandle, this);
        ROS_ERROR_STREAM_COND(!_srvCleanup, "The cleanup service is empty!");

        _srvRecover = _nodeHandle.advertiseService("recover", &CarosNodeServiceInterface::recoverHandle, this);
        ROS_ERROR_STREAM_COND(!_srvRecover, "The recover service is empty!");

        if (_nodeStatePublisher && _srvStop && _srvStart && _srvConfigure && _srvCleanup && _srvRecover) {
            /* Everything seems to be properly initialised */
        } else {
            ROS_FATAL_STREAM("One or more of the ROS publishers or services could not be properly initialised.");
            return false;
        }

        return true;
    }

    bool CarosNodeServiceInterface::stopHandle(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
        return stopNode();
    }

    bool CarosNodeServiceInterface::startHandle(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
        return startNode();
    }

    bool CarosNodeServiceInterface::configureHandle(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
        return configureNode();
    }

    bool CarosNodeServiceInterface::cleanupHandle(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
        return cleanupNode();
    }

    bool CarosNodeServiceInterface::recoverHandle(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
        return recoverNode();
    }

    void CarosNodeServiceInterface::publishNodeState(const bool stateChanged) {
        caros_common_msgs::CarosNodeState state;
        state.state = CarosStateString[_nodeState];
        state.inError = _nodeState == INERROR || _nodeState == INFATALERROR;
        if (state.inError) {
            state.errorMsg = _errorMsg;
            state.errorCode = _errorCode;
        }
        /* The errorMsg is not being cleared when the state no longer is in error, this is intended to provide a minor error log / history of errors.
         * TODO: Provide a history/log of the last N errors together with some info such as a timestamp (maybe how long the node was in the error state) and similar.
         */

        state.changedEvent = stateChanged;

        _nodeStatePublisher.publish( state );
    }
} // namespace caros
