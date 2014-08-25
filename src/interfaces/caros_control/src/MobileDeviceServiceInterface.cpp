/**/
#include <marvin_common_rw/MobileDeviceServiceInterface.hpp>

using namespace rw::common;

MobileDeviceServiceInterface::MobileDeviceServiceInterface(const std::string& service_name)
{
    _nodeHnd = ownedPtr( new ros::NodeHandle(service_name) );

    //_robotStatePublisher = _nodeHnd.advertise<SDHState>("SDHState", 5);
    //_srvStop = _nodeHnd.advertiseService("stop", &SDHServiceInterface::stopHandle, this);
}

