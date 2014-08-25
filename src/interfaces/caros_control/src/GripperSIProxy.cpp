#include <caros/GripperSIProxy.hpp>

#include <caros/GripperServiceInterface.hpp> /* provides GRIPPER_SERVICE_INTERFACE_SUB_NAMESPACE */
#include <caros/common.hpp>
#include <caros/exceptions.hpp>

#include <caros_control_msgs/GripperState.h>

/* TODO:
 * - Does it make sense to have a nodehandle or just use the non-object/instance versions of the function calls?
 *  ^--< With this simple implementation, where the subscribed callback will only be called when the user of this proxy calls spin or otherwise invoke the callback handling, then it makes plenty of sense to enforce the user to provide a nodehandle - otherwise this functionality will not work as intended.
 *  ^-- This also makes it the user's responsibility to ensure that the calls to getTimeStamp() and other interesting getters are done on the same received GripperState i.e. not calling any spin or processing of the subscribing callback queue... otherwise the data may be inconsistent.
 *  ^-- If using a thread to run the handleGripperState callback, then it would make sense to expose the reported gripperState to the user or create a struct holding the same fields just with converted types such as rw::math::Q instead of caros_common_msgs/Q.
 *
 * - Would it make sense to default to persistent connections? and automatic reconnection?
 *
 * - See http://wiki.ros.org/roscpp/Overview/Callbacks%20and%20Spinning for more information on turning this proxy into a "selfhosted" ros node - so it spawns a thread that spins and handle the subscribed queue/callback - this can make it possible to fully eliminate the knowledge of ROS from the user of this proxy (given that the ros::Time (header, timestamp) type is eliminated by finding another type to represent the timestamps).
 * ^-- Also document these assumptions, so that users of this proxy easily understand how to properly use it.
 *
 * What to do when no gripper state data has been acquired yet?
 *   - Simply throw an exception stating that the data is invalid?
 *   - or how to properly signal that the data has not been updated at all?
 *   - what about timestamps?
 *
 * How to diagnose if the subscription is still valid and reinitiate a subscription? [ use _subGripperState() != 0 or similar, but reinitiating requires creating a new object or is that supposed to never ever happen so the functionality is missing? [ possibly interesting functions: getNumPublishers() and/or getTopic() ... ]
 * - Should there be a function to verify the subscription state/condition, and/or have every function that is being invoked verify the state/condition of this object? Or should it just be the getters that verify that the subscription is still active?
 *
 * The service calls can be extended with http://docs.ros.org/api/roscpp/html/classros_1_1ServiceClient.html : waitForExistence(...) - taking an optional parameter specifying the timeout or just a boolean specifying whether or not to block until the service becomes available...
 *
 * The user should have easy access to the error information regarding the state of the gripper / gripper caros node.
 *
 * Add more getters to allow the user to obtain more information from the reported gripper state. Maybe even allow the user access to the whole gripper state with ROS types (the user would then have to properly handle the type conversions). Also see comment above about creating a struct holding the converted gripperState fields.
 *
 * Provide synchronous versions of the service calls such as moveQ and gripQ. This would make a blocking call waiting for the reported gripper state to show the proper values, but handleGripperState would never be run with the current setup - requiring the subscription callback to be handled in its own thread, as described above (in one of the TODO comments).
 */

using namespace caros;

GripperSIProxy::GripperSIProxy(ros::NodeHandle nodehandle, const std::string& devname):
    _nodeHnd(nodehandle)
{
    std::ostringstream rosNamespace;
    rosNamespace << "/" << devname << "/" << GRIPPER_SERVICE_INTERFACE_SUB_NAMESPACE;

    _srvMoveQ = _nodeHnd.serviceClient<caros_control_msgs::GripperMoveQ>(rosNamespace.str() + "/move_q");
    _srvGripQ = _nodeHnd.serviceClient<caros_control_msgs::GripperGripQ>(rosNamespace.str() + "/grip_q");
    _srvSetForceQ = _nodeHnd.serviceClient<caros_control_msgs::GripperSetForceQ>(rosNamespace.str() + "/set_force_q");
    _srvSetVelocityQ = _nodeHnd.serviceClient<caros_control_msgs::GripperSetVelocityQ>(rosNamespace.str() + "/set_velocity_q");
    _srvStopMovement = _nodeHnd.serviceClient<caros_control_msgs::GripperStopMovement>(rosNamespace.str() + "/stop_movement");

    /* TODO:
     * Make the queue size into a parameter that can be configured - (hardcoded to 1 here)
     */
    _subGripperState = _nodeHnd.subscribe("GripperState", 1, &GripperSIProxy::handleGripperState, this);
}

GripperSIProxy::~GripperSIProxy() {
    /* Empty */
}

bool GripperSIProxy::moveQ(const rw::math::Q& q)
{
    bool srvCallSuccess = false;
    caros_control_msgs::GripperMoveQ srv;
    srv.request.q = caros::toRos(q);

    if (! _srvMoveQ.exists()) {
        THROW_CAROS_UNAVAILABLE_SERVICE("The service " << _srvMoveQ.getService() << " does not exist.");
    }
    
    srvCallSuccess = _srvMoveQ.call(srv);
    if (!srvCallSuccess) {
        THROW_CAROS_BAD_SERVICE_CALL("An error happened while calling the service " << _srvMoveQ.getService());
    }

    return srv.response.success;
}

bool GripperSIProxy::gripQ(const rw::math::Q& q)
{
    bool srvCallSuccess = false;
    caros_control_msgs::GripperGripQ srv;
    srv.request.q = caros::toRos(q);

    if (! _srvGripQ.exists()) {
        THROW_CAROS_UNAVAILABLE_SERVICE("The service " << _srvGripQ.getService() << " does not exist.");
    }
    
    srvCallSuccess = _srvGripQ.call(srv);
    if (!srvCallSuccess) {
        THROW_CAROS_BAD_SERVICE_CALL("An error happened while calling the service " << _srvGripQ.getService());
    }

    return srv.response.success;
}

bool GripperSIProxy::setForceQ(const rw::math::Q& q)
{
    bool srvCallSuccess = false;
    caros_control_msgs::GripperSetForceQ srv;
    srv.request.force = caros::toRos(q);

    if (! _srvSetForceQ.exists()) {
        THROW_CAROS_UNAVAILABLE_SERVICE("The service " << _srvSetForceQ.getService() << " does not exist.");
    }
    
    srvCallSuccess = _srvSetForceQ.call(srv);
    if (!srvCallSuccess) {
        THROW_CAROS_BAD_SERVICE_CALL("An error happened while calling the service " << _srvSetForceQ.getService());
    }

    return srv.response.success;
}

bool GripperSIProxy::setVelocityQ(const rw::math::Q& q)
{
    bool srvCallSuccess = false;
    caros_control_msgs::GripperSetVelocityQ srv;
    srv.request.velocity = caros::toRos(q);

    if (! _srvSetVelocityQ.exists()) {
        THROW_CAROS_UNAVAILABLE_SERVICE("The service " << _srvSetVelocityQ.getService() << " does not exist.");
    }
    
    srvCallSuccess = _srvSetVelocityQ.call(srv);
    if (!srvCallSuccess) {
        THROW_CAROS_BAD_SERVICE_CALL("An error happened while calling the service " << _srvSetVelocityQ.getService());
    }

    return srv.response.success;
}

bool GripperSIProxy::stopMovement()
{
    bool srvCallSuccess = false;
    caros_control_msgs::GripperStopMovement srv;

    if (! _srvStopMovement.exists()) {
        THROW_CAROS_UNAVAILABLE_SERVICE("The service " << _srvStopMovement.getService() << " does not exist.");
    }
    
    srvCallSuccess = _srvStopMovement.call(srv);
    if (!srvCallSuccess) {
        THROW_CAROS_BAD_SERVICE_CALL("An error happened while calling the service " << _srvStopMovement.getService());
    }

    return srv.response.success;
}

rw::math::Q GripperSIProxy::getQ() {
    boost::mutex::scoped_lock lock(_pSV);
    return caros::toRw(_pSV_gripperState.q);
}

rw::math::Q GripperSIProxy::getQd() {
    boost::mutex::scoped_lock lock(_pSV);
    return caros::toRw(_pSV_gripperState.dq);
}

rw::math::Q GripperSIProxy::getForce() {
    boost::mutex::scoped_lock lock(_pSV);
    return caros::toRw(_pSV_gripperState.force);
}

ros::Time GripperSIProxy::getTimeStamp() {
    boost::mutex::scoped_lock lock(_pSV);
    return _pSV_gripperState.header.stamp;
}

void GripperSIProxy::handleGripperState(const caros_control_msgs::GripperState& state) {
	boost::mutex::scoped_lock lock(_pSV);
	_pSV_gripperState = state;
}
