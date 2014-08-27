#include "UniversalRobots.hpp"

#include <rw/common/PropertyMap.hpp>
#include <rw/loaders/xml/XMLPropertyLoader.hpp>
#include <rw/loaders/WorkCellLoader.hpp>
#include <rwhw/netft/FTCompensation.hpp>
#include <boost/bind.hpp>
#include <caros/common.hpp>


using namespace rw::loaders;
using namespace rw::models;
using namespace rw::common; 
using namespace rw::loaders;
using namespace rwhw;


int main(int argc, char **argv)
{
	ros::init(argc, argv, "universalrobots");

	ros::NodeHandle nh("~");
	std::string name = nh.getNamespace();
	std::string propertyFile;
	nh.getParam("/properties", propertyFile);
	ROS_INFO_STREAM("properties: " << propertyFile);
	PropertyMap properties = XMLPropertyLoader::load( propertyFile );
	if (!properties.has("Name")) {
		ROS_ERROR("No property named 'Name' found in property file");
		return 1;
	}

	WorkCell::Ptr workcell;
	try {
		workcell = caros::getWorkCell();
	} catch (const Exception& exp) {
	    ROS_ERROR_STREAM("Unable to open file '"<<argv[1]<<"'");
	    ROS_ERROR_STREAM("Error: "<<exp.what());
		return 1;
	}

	if (workcell == NULL) {
		ROS_ERROR("You did not provide a workcell or it could not be loaded.");
		ROS_ERROR("Please provide a workcell!");
		return 1;
	}

	ROS_INFO("WorkCell loaded!");
	std::string deviceName = properties.get<std::string>("Name");
	Device::Ptr dev = workcell->findDevice(deviceName);
	if (dev == NULL) {
		ROS_ERROR_STREAM("Unable to find device "<<deviceName<<" in work cell");
		return 1;
	}
	ROS_INFO("WorkCell loaded! Starting ur component.");

    FTCompensation::Ptr emptyPtr;
    UniversalRobots ur(workcell, properties, 125, "", emptyPtr);
    ur.run();

    return 0;
}

