#ifndef _GRIPPERSIPROXY_HPP_
#define _GRIPPERSIPROXY_HPP_

#include <caros_control_msgs/GripperState.h>

#include <rw/math/Q.hpp>

#include <ros/ros.h>

#include <boost/thread/mutex.hpp>

namespace caros {
    /**
     * @brief This class implements a C++ proxy to control and read data from a GripperServiceInterface.
     */
    class GripperSIProxy {
    public:
        // typedef rw::common::Ptr<GripperSIProxy> Ptr;

        /**
         * @brief Constructor
         * @param[in] nodehandle
         * @param[in] devname The name of the CAROS gripper node
         */
        GripperSIProxy(ros::NodeHandle nodehandle, const std::string& devname);

        //! Destructor
        virtual ~GripperSIProxy();

        /* TODO:
         *   What about blocking and non-blocking calls (given we don't use the action library)?
         *
         *   Maybe in the detailed descriptions also (if the gripper service interface ends up supporting it) refer to the functionality that allows for obtaining the supported limits of the force/current, velocities and positions
         *
         *   What promises are we making with respect to accepting and/or completing commands - is it only the setForceQ and setVelocityQ commands that are guaranteed to be successfully completed if true is returned? [ related to synchronous and asynchronous commands ]
         */

        /**
         * @brief Move gripper to configuration
         * @param[in] q The target configuration
         * @returns a boolean indicating if the gripper accepted the command.
         * @throws unavailableService when the command is currently unavailable. This indicates that the connection to the gripper is not fully working, or the gripper has not announced this service yet.
         * @throws badServiceCall when an error happened while communicating with the gripper.
         */
        bool moveQ(const rw::math::Q& q);

        /**
         * @brief Grasp with the given configuration
         * @param[in] q The target configuration to use for the grasp
         * @returns a boolean indicating if the gripper accepted the command.
         * @throws unavailableService when the command is currently unavailable. This indicates that the connection to the gripper is not fully working, or the gripper has not announced this service yet.
         * @throws badServiceCall when an error happened while communicating with the gripper.
         *
         * [Detailed description] TODO: should describe something about how grasping is different than move (such as the hand (if it supports it) will continue to apply force trying to get into the target configuration)
         */
        bool gripQ(const rw::math::Q& q);

        /**
         * @brief Set the force configuration for the gripper
         * @param[in] q The force configuration
         * @returns a boolean indicating if the gripper successfully completed the command.
         * @throws unavailableService when the command is currently unavailable. This indicates that the connection to the gripper is not fully working, or the gripper has not announced this service yet.
         * @throws badServiceCall when an error happened while communicating with the gripper.
         */
        bool setForceQ(const rw::math::Q& q);

        /**
         * @brief Set the velocity configuration for the gripper
         * @param[in] q The velocity configuration
         * @returns a boolean indicating if the gripper successfully completed the command.
         * @throws unavailableService when the command is currently unavailable. This indicates that the connection to the gripper is not fully working, or the gripper has not announced this service yet.
         * @throws badServiceCall when an error happened while communicating with the gripper.
         */
        bool setVelocityQ(const rw::math::Q& q);

        /**
         * @brief Stop the gripper's movement(s)
         * @returns a boolean indicating if the gripper accepted the command.
         * @throws unavailableService when the command is currently unavailable. This indicates that the connection to the gripper is not fully working, or the gripper has not announced this service yet.
         * @throws badServiceCall when an error happened while communicating with the gripper.
         */
        bool stopMovement();

        /**
         * @brief Get the last reported joint configuration \b Q from the gripper
         * @returns The reported joint configuration of the gripper
         */
        rw::math::Q getQ();

        /**
         * @brief Get the last reported velocities from the gripper
         * @returns The reported velocities of the gripper
         */
        /* TODO: getVelocity() instead of getQd()? or as an alias/alternative? */
        rw::math::Q getQd();

        /**
         * @brief Get the last reported forces from the gripper
         * @returns The reported forces of the gripper
         *
         * TODO: Currently it is probably just the (electrical) currents that will be returned, as there is no implemented conversion to get the actual forces instead
         */
        rw::math::Q getForce();

        /**
         * @brief Get the timestamp that is associated with the last reported gripper state
         * @returns The timestamp of the last reported state of the gripper
         */
        ros::Time getTimeStamp();

/* TODO:
 *   Should something be protected instead of private?
 *   - What use cases would benefit from being able to access the service clients or the nodeHandle?
 */
    private:
        // services
        ros::ServiceClient _srvMoveQ;
        ros::ServiceClient _srvGripQ;
        ros::ServiceClient _srvSetForceQ;
        ros::ServiceClient _srvSetVelocityQ;
        ros::ServiceClient _srvStopMovement;

        // states
        ros::Subscriber _subGripperState;

    private:
        void handleGripperState(const caros_control_msgs::GripperState& state);

        ros::NodeHandle _nodeHnd;

        // protectStateVariable
        boost::mutex _pSV;

        caros_control_msgs::GripperState _pSV_gripperState;
    };
}
#endif /* _GRIPPERSIPROXY_HPP_ */
