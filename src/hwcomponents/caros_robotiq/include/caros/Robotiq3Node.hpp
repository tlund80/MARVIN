/**/
#ifndef CAROS_ROBOTIQ3NODE_HPP
#define CAROS_ROBOTIQ3NODE_HPP

#include <caros/GripperServiceInterface.hpp>
#include <caros/CarosNodeServiceInterface.hpp>

#include <rwhw/robotiq/Robotiq3.hpp>

#include <rw/math/Transform3D.hpp>
#include <rw/math/Q.hpp>

/**
 * @brief Ros node for controlling Robotiq-3 gripper.
 */
class Robotiq3Node: public caros::CarosNodeServiceInterface, public caros::GripperServiceInterface{
public:

    enum ERRORCODE {
        CONNECTION_ERROR = 1 //! Connection to robotiq hand was not possible
    };

    //! constructor
    Robotiq3Node(const std::string& name);

    //! destructor
    virtual ~Robotiq3Node(){};

    //! @copydoc caros::GripperServiceInterface::moveQ
    bool moveQ(const rw::math::Q& q);

    //! @copydoc caros::GripperServiceInterface::moveQ
    bool gripQ(const rw::math::Q& q);

    //! @copydoc caros::GripperServiceInterface::setForceQ
    bool setForceQ(const rw::math::Q& q);

    //! @copydoc caros::GripperServiceInterface::setVelocityQ
    bool setVelocityQ(const rw::math::Q& q);

    //! @copydoc caros::GripperServiceInterface::stopMovement
    bool stopMovement(void);

protected:
    // hooks implemented from base class

    void runLoopHook();
    bool configureHook();
    bool cleanupHook();
    bool startHook();
    bool stopHook();
    bool recoverHook();

protected:
    typedef enum{MOVE,GRIP,STOP} CmdType;
    int _lastCmd;
    ros::Time _lastLoopTime;
    rw::math::Q _lastQ;
    std::string _ip;
    int _port;
    std::string _nodeName;
private:
    rw::common::Ptr<rwhw::Robotiq3> _robotiq;
};


#endif //#ifndef SDH_HPP
