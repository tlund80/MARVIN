#ifndef CAROS_CAROSNODESERVICEINTERFACE_HPP_
#define CAROS_CAROSNODESERVICEINTERFACE_HPP_

#include <ros/ros.h>

#include <std_srvs/Empty.h>

#include <string>

#define CAROS_NODE_SERVICE_INTERFACE_SUB_NAMESPACE "caros_node"

/************************************************************************
 * TODO:
 * - Add apidoc documentation
 * - Error codes: Make use of a "singleton counter" and the parameter
 *                server to handle the distribution of error codes and
 *                their corresponding human-friendly description.
 ************************************************************************/

/**
 * @brief Emit an CAROS node error. It will also be emitted to ROS_ERROR
 *
 * \b ostreamExpression is an expression that is fed to an output stream. Example:
\code
CAROS_ERROR("The value of x is " << x << ". x should be less than zero.", 2);
\endcode
 *
 */
#define CAROS_ERROR(ostreamExpression, errorCode) do { \
    std::stringstream ERROR__stream;                                           \
    ERROR__stream << ostreamExpression;                                        \
    ROS_ERROR_STREAM("CarosNodeError: " << ERROR__stream.str() << "; error code: " << errorCode); \
    error(ERROR__stream.str(), errorCode );                                     \
    } while (0)

/**
 * @brief Emit an CAROS node fatal error. It will also be emitted to ROS_ERROR
 *
 * \b ostreamExpression is an expression that is fed to an output stream. Example:
\code
CAROS_FATALERROR("The value of x is " << x << ". x must not be less than zero.", 5);
\endcode
 *
 */
#define CAROS_FATALERROR(ostreamExpression, errorCode) do { \
    std::stringstream FATALERROR__stream;                                           \
    FATALERROR__stream << ostreamExpression;                                        \
    ROS_ERROR_STREAM("CarosNodeFatalError: " << FATALERROR__stream.str() << "; error code: " << errorCode); \
    fatalError(FATALERROR__stream.str(), errorCode );                                     \
    } while (0)


namespace caros {
    enum CAROS_NODE_ERRORCODE { CAROS_NODE_NO_ERROR_CODE_SUPPLIED = 0 };

    /**
     * @brief A node service interface that defines a simple statemachine from which
     * the node can be controlled.
     *
     * There are 5 states: init, stopped, running, error, fatalerror
     *
     * The following transitions are legal
     * init: (configure)
     * ->fatalerror (through configure)
     * ->stopped (through configure)
     * stopped: (cleanup, start, error, ferror)
     * ->init (through cleanup)
     * ->running (through start)
     * ->error( through error)
     * ->fatalerror(through ferror)
     * running: (stop,error,ferror)
     * ->stopped (through stop)
     * ->error (through error)
     * ->fatalerror (through ferror)
     * error: (cleanup, retry)
     * ->running(retry)
     * ->init(through cleanup)
     * ->stopped(through retry)
     *
     *
     *
     *
     */
    class CarosNodeServiceInterface {
    public:
        /**
         * @brief constructor.
         * @param[in] nodehandle The nodehandle to use for ROS services and publishers.
         * @param[in] loopRateFrequency Optional parameter that specifies the frequency [Hz] of this ROS node - see setLoopRateFrequency() for more information.
         */
        CarosNodeServiceInterface(const ros::NodeHandle& nodehandle, const double loopRateFrequency = 30);

        /**
         * @brief virtual destructor
         */
        virtual ~CarosNodeServiceInterface();

        /**
         * @brief Start the CAROS node
         *
         * Invoke this function to hand over the control to the CAROS node statemachine.
         * This is a blocking function that will return when ROS is being told to shutdown.
         */
        void start();

        /**
         * @brief The states that the CAROS node can be in
         */
        typedef enum {PREINIT,STOPPED,RUNNING,INERROR,INFATALERROR} NodeState;

    protected:

        /* TODO:
         * Should the Hook description mention something about deriving/child classes?
         * These apidocs contains a lot of duplicated information - should that be deduplicated or is there a better alternative?
         */
        /**
         * @name ROS Service Hooks
         * @brief These hooks needs to be implemented in the deriving node, allowing for a common interface for controlling CAROS nodes.
         */
        /** @{ */
        /**
         * @brief This is called as part of the configuration step that is invoked through the ROS service "configure".
         *
         * This hook should be used to initialise interfaces (e.g. the CAROS interface GripperServiceInterface) and advertise ROS services and publishers that are specific to the node.
         * If an error occurs, then fatalError() should be called with an appropriate error message and error code, and false returned.
         */
        virtual bool configureHook() = 0;

        /**
         * @brief This is called as part of the cleanup step that is invoked through the ROS service "cleanup".
         *
         * This hook should be used to properly cleanup interfaces (e.g. the CAROS interface GripperServiceInterface) and shut down ROS services and publishers that are specific to the node.
         */
        virtual bool cleanupHook() = 0;

        /**
         * @brief This is called when the node is transitioning to the RUNNING state, invoked through the ROS service "start".
         *
         * This hook should be used to establish connections to the hardware and activate the hardware.
         */
        virtual bool startHook() = 0;
    
        /**
         * @brief This is called when stopping the node, invoked through the ROS service "stop".
         *
         * This hook should be used to deactivate the hardware and shut down any established connections.
         */
        virtual bool stopHook() = 0;

        /**
         * @brief This is called as part of a recovery process that is invoked through the ROS service "recover".
         *
         * This hook should be used to perform any necessary steps to recover from the error.
         * The design of the recovery process is to be considered incomplete. Some things are missing such as the ability to properly see what error the node has been told to recover from (it's available in the CarosNodeService, but how to properly use it when recovering is undecided).
         */
        virtual bool recoverHook() = 0;
        /** @} */

        /**
         * @name Loop Hooks
         *
         * The loop hook corresponding to the current state will be invoked at the frequency specified when calling the constructor CarosNodeServiceInterface(const ros::NodeHandle& nodehandle, const double loopRateFrequency) or set through setLoopRateFrequency().
         */
        /** @{ */
        virtual void runLoopHook() = 0;

        virtual void initLoopHook() { /* Empty */ };
        virtual void stoppedLoopHook() { /* Empty */ };
        virtual void errorLoopHook() { /* Empty */ };
        virtual void fatalErrorLoopHook() { /* Empty */ };
        /** @} */

        void error(const std::string& msg, const int64_t errorCode = CAROS_NODE_NO_ERROR_CODE_SUPPLIED);
        void fatalError(const std::string& msg, const int64_t errorCode = CAROS_NODE_NO_ERROR_CODE_SUPPLIED);

        NodeState getState() { return _nodeState; }
        NodeState getPreviousState() { return _previousState; }

        /** @{ */
        bool isInRunning() { return _nodeState == RUNNING; }
        bool isInStopped() { return _nodeState == STOPPED; }
        bool isInInit() { return _nodeState == PREINIT; }
        bool isInError() { return _nodeState == INERROR; }
        bool isInFatalError() { return _nodeState == INFATALERROR; }
        /** @} */

        /**
         * @brief Change the frequency of this ROS node.
         *
         * @param[in] loopRate The new frequency [Hz].
         *
         * Change how often this node is supposed to execute ROS service callbacks and publish CAROS node messages.
         * A very small value or a negative value will (according to the current roscpp implementation) cause this ROS node to process the service callbacks and publishers as fast as possible.
         */
        void setLoopRateFrequency(const double frequency);

        /**
         * @brief Get the frequency of this ROS node.
         *
         * @returns The frequency [Hz].
         */
        double getLoopRateFrequency() { return _loopRateFrequency; }

    private:
        /**
         * @brief private default constructor.
         * This is declared as private to enforce deriving classes to call an available public constructor and enforce that the ROS services are properly initialised.
         */
        CarosNodeServiceInterface();

        /** @{ */
        /**
         * @brief Configure the CAROS node.
         *
         * This initialises the ROS services and advertise them, bringing the node into a working state.
         *
         * @pre In PREINIT state
         * @post Success: in STOPPED state
         * @post Failure: in INFATALERROR state
         */
        bool configureNode();

        /**
         * @brief Clean up the CAROS node.
         *
         * This shuts down the ROS services and cleans up allocated resources, bringing the node into the PREINIT state.
         * 
         * If the node is in RUNNING state then stopNode() will be invoked first.
         *
         * @pre In any state
         * @post Success: in PREINIT state
         * @post Failure: in INFATALERROR state
         */
        bool cleanupNode();

        /**
         * @brief Start the CAROS node.
         *
         * This puts the node in a running state, where connections to the hardware have been established and the hardware activated.
         *
         * @pre In STOPPED state
         * @post Success: in RUNNING state
         * @post Failure: in one of the error states, depending on the severity of the failure
         */
        bool startNode();

        /**
         * @brief Stop the CAROS node.
         *
         * This stops the node (e.g. leaving the running state), which should deactivate the hardware and shut down the previously established connections.
         *
         * @pre In RUNNING or INERROR state
         * @post Success: in STOPPED state
         * @post Failure: in one of the error states, depending on the severity of the failure
         */
        bool stopNode();

        /**
         * @brief Recover from an error.
         *
         * If it's possible to recover then the node will go into the state that the node was in before entering the error state.
         *
         * @pre In INERROR state
         * @post Success: in the previous state
         * @post Failure: In INERROR state [ TODO: Settle on a strategy possibly involving INFATALERROR ]
         */
        bool recoverNode();
        /** @} */

        /**
         * @brief internal GripperServiceInterface function to properly advertise the services using ROS and setup the callbacks.
         *
         * This should not be called directly.
         */
        bool initCarosNode();

        /** @{ */
        /**
         * @brief ROS service wrapper for stopNode().
         */
        bool stopHandle(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
        /**
         * @brief ROS service wrapper for startNode().
         */
        bool startHandle(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
        /**
         * @brief ROS service wrapper for configureNode().
         */
        bool configureHandle(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
        /**
         * @brief ROS service wrapper for cleanupNode().
         */
        bool cleanupHandle(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
        /**
         * @brief ROS service wrapper for recoverNode().
         */
        bool recoverHandle(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
        /** @} */

        void changeState(const NodeState newState);

        void publishNodeState(const bool stateChanged = false);

    private:
        ros::NodeHandle _nodeHandle;

        ros::Publisher _nodeStatePublisher;

        ros::ServiceServer _srvStop;
        ros::ServiceServer _srvStart;
        ros::ServiceServer _srvConfigure;
        ros::ServiceServer _srvCleanup;
        ros::ServiceServer _srvRecover;

        NodeState _nodeState;
        NodeState _previousState;

        /* TODO:
         * Should the _loopRateFrequency be settable through the CarosNodeServiceInterface that is exposed as ROS services?
         */
        double _loopRateFrequency;
        ros::Rate _loopRate;

        std::string _errorMsg;
        /* Using int64_t because it's highly related with the type specified in the caros_common_msgs::CarosNodeState message */
        int64_t _errorCode;

    };
} // namespace
#endif
