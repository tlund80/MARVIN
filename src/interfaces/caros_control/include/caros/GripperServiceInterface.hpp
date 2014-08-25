#ifndef CAROS_GripperServiceInterface_HPP_
#define CAROS_GripperServiceInterface_HPP_

#include <rw/math/Q.hpp>

#include <ros/ros.h>

#include <caros_control_msgs/GripperMoveQ.h>
#include <caros_control_msgs/GripperGripQ.h>
#include <caros_control_msgs/GripperSetForceQ.h>
#include <caros_control_msgs/GripperSetVelocityQ.h>
#include <caros_control_msgs/GripperStopMovement.h>

#include <string>

/* Always publish the latest gripper state */
#define GRIPPER_STATE_PUBLISHER_QUEUE_SIZE 1
#define GRIPPER_SERVICE_INTERFACE_SUB_NAMESPACE "caros_gripper_service_interface"

namespace caros {

    /**
     * @brief This is the gripper interface. It defines the
     * minimum interface that a configuration based robotic grasping device needs
     * to implement.
     *
     * In ROS the namespace of the node is used and it is important that
     * not two GripperServiceInterfaces are running in the same namespace.
     *
     */
    class GripperServiceInterface {
    public:
        //!
        typedef rw::common::Ptr<GripperServiceInterface> Ptr;
        /**
         * @brief constructor.
         * @param[in] nodehandle the nodehandle to use for services.
         */
        GripperServiceInterface(const ros::NodeHandle& nodehandle);

        /**
         * @brief virtual destructor
         */
        virtual ~GripperServiceInterface();

        /* FIXME: apidoc documentation */
        bool configureGripperService();
        /* FIXME: apidoc documentation */
        bool cleanupGripperService();

        /**
         * @brief signal the gripper to move into a specific configuration Q.
         * @param q
         * @return
         */
        virtual bool moveQ(const rw::math::Q& q) = 0;

        /**
         * @brief signal the gripper to move into a specific configuration Q. The gripper will not show an error in its state if the configuration Q can not be reached.
         * @param q
         * @return
         */
        virtual bool gripQ(const rw::math::Q& q) = 0;

        /**
         * @brief set the desired force configuration that the gripper should use.
         * @param q
         * @return
         */
        virtual bool setForceQ(const rw::math::Q& q) = 0;

        /**
         * @brief set the desired velocity configuration that the gripper should use.
         * @param q
         * @return
         */
        virtual bool setVelocityQ(const rw::math::Q& q) = 0;

        /**
         * @brief signal the gripper to stop all its movements.
         * It should not power down the gripper and/or disconnect from the gripper.
         * @return
         */
        virtual bool stopMovement(void) = 0;

    protected:

        /**
         * @brief publish the state of the gripper. Uses GripperState messages
         * @param[in] q joint configuration
         * @param[in] dq joint velocity
         * TODO: Add documentation for the rest of the parameters
         */
        void publishState(const rw::math::Q& q,
                          const rw::math::Q& dq,
                          const rw::math::Q& jointforce,
                          bool isMoving,
                          bool isBlocked,
                          bool isStopped,
                          bool isEstopped);


    private:
        /**
         * @brief private default constructor.
         * This is declared as private to enforce deriving classes to call an available public constructor and enforce that the ROS services are properly initialised.
         */
        GripperServiceInterface();

        /**
         * @brief internal GripperServiceInterface function to properly advertise the services using ROS and setup the callbacks.
         * This should not be called directly.
         */
        bool initGripperService();


        /* - these functions should be grouped together in the documentation t shortly describe that these are converting from ROS types to e.g. RobWork types according to what the interface expects - */
        bool moveQHandle(caros_control_msgs::GripperMoveQ::Request& request,
                         caros_control_msgs::GripperMoveQ::Response& response);

        bool gripQHandle(caros_control_msgs::GripperGripQ::Request& request,
                         caros_control_msgs::GripperGripQ::Response& response);

        bool setForceQHandle(caros_control_msgs::GripperSetForceQ::Request& request,
                             caros_control_msgs::GripperSetForceQ::Response& response);

        bool setVelocityQHandle(caros_control_msgs::GripperSetVelocityQ::Request& request,
                                caros_control_msgs::GripperSetVelocityQ::Response& response);

        bool stopMovementHandle(caros_control_msgs::GripperStopMovement::Request& request,
                                caros_control_msgs::GripperStopMovement::Response& response);

    private:
        ros::NodeHandle _nodeHandle;

        ros::Publisher _gripperStatePublisher;

        ros::ServiceServer _srvMoveQ;
        ros::ServiceServer _srvGripQ;
        ros::ServiceServer _srvSetForceQ;
        ros::ServiceServer _srvSetVelocityQ;
        ros::ServiceServer _srvStopMovement;

        /************************************************************************
         * Notes:
         * If it is required to know whether this object is configured/initialised then a variable/state needs to be implemented. Furthermore extra care should be taken to make sure that this object is not left in an "undefined" state. This would happen when some of the ros services or publishers fails to be initialised or shutdown properly.
         ************************************************************************/
    };
} // namespace
#endif
