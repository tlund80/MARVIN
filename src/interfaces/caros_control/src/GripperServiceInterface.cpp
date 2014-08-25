#include <caros/GripperServiceInterface.hpp>
#include <caros/common.hpp>

#include <caros_control_msgs/GripperState.h>

#include <rw/math/Q.hpp>

#include <ros/ros.h>

#include <string>

using namespace caros;

GripperServiceInterface::GripperServiceInterface(const ros::NodeHandle& nodehandle) :
        _nodeHandle(nodehandle, GRIPPER_SERVICE_INTERFACE_SUB_NAMESPACE)
{
    /* Do nothing */
}

GripperServiceInterface::GripperServiceInterface()
{
    /* Do nothing */
    ROS_FATAL_STREAM(
            "The empty constructor of the GripperServiceInterface should never be called! This is undefined behaviour.");
}

GripperServiceInterface::~GripperServiceInterface()
{
    /* Currently no special things to clean up */
}

void GripperServiceInterface::publishState(const rw::math::Q& q, const rw::math::Q& dq, const rw::math::Q& jointforce,
                                           bool isMoving, bool isBlocked, bool isStopped, bool isEstopped)
{
    caros_control_msgs::GripperState state;

    state.header.stamp = ros::Time::now();

    state.q = caros::toRos(q);
    state.dq = caros::toRos(dq);
    state.force = caros::toRos(jointforce);
    state.isMoving = isMoving;
    state.isBlocked = isBlocked;
    state.isStopped = isStopped;
    state.estopped = isEstopped;

    if (_gripperStatePublisher) {
        _gripperStatePublisher.publish(state);
    } else {
        ROS_ERROR_STREAM(
                "The _gripperStatePublisher is empty - trying to publish gripper state with a non-working GripperSreviceInterface object.");
    }
}

bool GripperServiceInterface::configureGripperService()
{
    return initGripperService();
}

bool GripperServiceInterface::cleanupGripperService()
{
    if (_gripperStatePublisher) {
        _gripperStatePublisher.shutdown();
    } else {
        ROS_ERROR_STREAM("While trying to cleanup the GripperService, _gripperStatePublisher was empty!");
    }
    if (_srvMoveQ) {
        _srvMoveQ.shutdown();
    } else {
        ROS_ERROR_STREAM("While trying to cleanup the GripperService, _srvMoveQ was empty!");
    }
    if (_srvGripQ) {
        _srvGripQ.shutdown();
    } else {
        ROS_ERROR_STREAM("While trying to cleanup the GripperService, _srvGripQ was empty!");
    }
    if (_srvSetForceQ) {
        _srvSetForceQ.shutdown();
    } else {
        ROS_ERROR_STREAM("While trying to cleanup the GripperService, _srvSetForceQ was empty!");
    }
    if (_srvSetVelocityQ) {
        _srvSetVelocityQ.shutdown();
    } else {
        ROS_ERROR_STREAM("While trying to cleanup the GripperService, _srvSetVelocityQ was empty!");
    }
    if (_srvStopMovement) {
        _srvStopMovement.shutdown();
    } else {
        ROS_ERROR_STREAM("While trying to cleanup the GripperService, _srvStopMovement was empty!");
    }

    return true;
}

bool GripperServiceInterface::initGripperService()
{
    if (_gripperStatePublisher || _srvMoveQ || _srvGripQ || _srvSetForceQ || _srvSetVelocityQ || _srvStopMovement) {
        ROS_WARN_STREAM(
                "Reinitialising one or more GripperServiceInterface services or publishers. If this is not fully intended then this should be considered a bug!");
    }

    _gripperStatePublisher = _nodeHandle.advertise<caros_control_msgs::GripperState>("GripperState",
                                                                                GRIPPER_STATE_PUBLISHER_QUEUE_SIZE);
    ROS_ERROR_STREAM_COND(!_gripperStatePublisher, "The GripperState publisher is empty!");

    _srvMoveQ = _nodeHandle.advertiseService("move_q", &GripperServiceInterface::moveQHandle, this);
    ROS_ERROR_STREAM_COND(!_srvMoveQ, "The move_q service is empty!");

    _srvGripQ = _nodeHandle.advertiseService("grip_q", &GripperServiceInterface::gripQHandle, this);
    ROS_ERROR_STREAM_COND(!_srvGripQ, "The grip_q service is empty!");

    _srvSetForceQ = _nodeHandle.advertiseService("set_force_q", &GripperServiceInterface::setForceQHandle, this);
    ROS_ERROR_STREAM_COND(!_srvSetForceQ, "The set_force_q service is empty!");

    _srvSetVelocityQ = _nodeHandle.advertiseService("set_velocity_q", &GripperServiceInterface::setVelocityQHandle,
                                                    this);
    ROS_ERROR_STREAM_COND(!_srvSetVelocityQ, "The set_velocity_q service is empty!");

    _srvStopMovement = _nodeHandle.advertiseService("stop_movement", &GripperServiceInterface::stopMovementHandle,
                                                    this);
    ROS_ERROR_STREAM_COND(!_srvStopMovement, "The stop_movement service is empty!");

    /* Verify that the various ROS services have actually been created properly */
    if (_gripperStatePublisher && _srvMoveQ && _srvGripQ && _srvSetForceQ && _srvSetVelocityQ && _srvStopMovement) {
        /* Everything seems to be properly initialised */
    } else {
        ROS_ERROR_STREAM(
                "The gripper service could not be properly initialised - one or more ros services or publishers failed to be properly initialised.");
        return false;
    }
    return true;
}


bool GripperServiceInterface::moveQHandle(caros_control_msgs::GripperMoveQ::Request& request,
                                          caros_control_msgs::GripperMoveQ::Response& response)
{
    response.success = moveQ(caros::toRw(request.q));
    return true;
}

bool GripperServiceInterface::gripQHandle(caros_control_msgs::GripperGripQ::Request& request,
                                          caros_control_msgs::GripperGripQ::Response& response)
{
    response.success = gripQ(caros::toRw(request.q));
    return true;
}

bool GripperServiceInterface::setForceQHandle(caros_control_msgs::GripperSetForceQ::Request& request,
                                              caros_control_msgs::GripperSetForceQ::Response& response)
{
    response.success = setForceQ(caros::toRw(request.force));
    return true;
}

bool GripperServiceInterface::setVelocityQHandle(caros_control_msgs::GripperSetVelocityQ::Request& request,
                                                 caros_control_msgs::GripperSetVelocityQ::Response& response)
{
    response.success = setVelocityQ(caros::toRw(request.velocity));
    return true;
}

bool GripperServiceInterface::stopMovementHandle(caros_control_msgs::GripperStopMovement::Request& request,
                                                 caros_control_msgs::GripperStopMovement::Response& response)
{
    response.success = stopMovement();
    return true;
}


