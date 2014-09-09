#ifndef DYNAMICWORKCELL_BUILDER_HPP_
#define DYNAMICWORKCELL_BUILDER_HPP_

/**
 * @file DynamicWorkCell_Builder.hpp
 */

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/optional.hpp>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>

#include <rw/math/Rotation3D.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/Q.hpp>
#include <rw/math/RPY.hpp>
#include <rw/kinematics/Frame.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/loaders/WorkCellLoader.hpp>
#include <rw/loaders/rwxml/DependencyGraph.hpp>
#include <rw/models/JointDevice.hpp>

#include <rw/common/macros.hpp>
#include <rw/common/StringUtil.hpp>
#include <rw/common/Log.hpp>
#include <rw/common/Ptr.hpp>

#include <rw/kinematics/Kinematics.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/Quaternion.hpp>
#include <rw/math/Constants.hpp>

#include <rwsim/dynamics/Body.hpp>
#include <rwsim/dynamics/Constraint.hpp>
#include <rwsim/dynamics/FixedBody.hpp>
#include <rwsim/dynamics/KinematicBody.hpp>
#include <rwsim/dynamics/RigidBody.hpp>
#include <rwsim/dynamics/RigidJoint.hpp>
#include <rwsim/dynamics/ControllerModel.hpp>
#include <rwsim/dynamics/SensorModel.hpp>

#include <rwsim/dynamics/KinematicDevice.hpp>
#include <rwsim/dynamics/RigidDevice.hpp>

#include <rwsim/dynamics/MaterialDataMap.hpp>
#include <rwsim/dynamics/ContactDataMap.hpp>
#include <rwsim/dynamics/DynamicUtil.hpp>
#include <rwsim/dynamics/SuctionCup.hpp>

#include <rwsim/sensor/TactileArraySensor.hpp>
#include <rwsim/sensor/BodyContactSensor.hpp>
#include <rwsim/sensor/SimulatedFTSensor.hpp>

#include <rwsim/control/SpringJointController.hpp>
#include <rwsim/control/SerialDeviceController.hpp>

#include <rw/geometry/GeometryUtil.hpp>
#include <rw/common/PropertyMap.hpp>
#include <rw/common/StringUtil.hpp>

#include <rwlibs/simulation/SimulatedController.hpp>
#include <rwlibs/simulation/SimulatedSensor.hpp>
#include <rwlibs/control/JointController.hpp>

#include <rw/geometry/Geometry.hpp>
#include <rw/loaders/GeometryFactory.hpp>
#include <rw/geometry/IndexedTriMesh.hpp>
#include <rw/geometry/TriangleUtil.hpp>

#include <rwsim/control/PDController.hpp>
#include <rwsim/control/SyncPDController.hpp>
#include <rwsim/control/VelRampController.hpp>
//#include <rwsim/control/TrajectoryController.hpp>
#include <rwsim/control/SuctionCupController.hpp>
#include <rwsim/control/PoseController.hpp>

typedef boost::property_tree::ptree PTree;
typedef PTree::iterator CI;

namespace dti{
namespace grasp_planning
{

class DynamicWorkCell_Builder
{  
  
  enum ObjectType{SOFTOBJECT = 0, HARDOBJECT};
  enum IntegratorType{EULER = 0};
  enum ODE_StepMethod{WorldStep = 0, Worldquickstep};
  

   public:

	DynamicWorkCell_Builder(rw::models::WorkCell::Ptr workcell);
	virtual ~DynamicWorkCell_Builder();
	
	void loadDynamicMaterialBase(const std::string& filename);
	void loadGripperDevice(const std::string& filename);
	rwsim::dynamics::DynamicWorkCell::Ptr getWorkcell(void);
	
	void setCollisionMargin(double margin) {defaultcolmargin =  margin; };
	void setRestitutionModel(std::string model) {defaultRestModel = model; };
	void setContactModel(std::string model){defaultContactModel = model; };
	void setGravity(rw::math::Vector3D<> gravity){_gravity = gravity;};
	void setAutoDisable(bool disable){autoDisable = disable; };
	void addRigidbody(rw::models::Object::Ptr object, rwsim::dynamics::BodyInfo info, bool estimateInertia = true);
	void addRigidbody(rw::models::Object::Ptr object);
	void addFixedbody(rw::models::Object::Ptr object, rwsim::dynamics::BodyInfo info);
	

	
  private:
	struct FrictionDataTmp {
        rwsim::dynamics::FrictionData data;
        std::string matA,matB;
    };

	struct ContactDataTmp {
        rwsim::dynamics::ContactDataMap::NewtonData data;
        std::string objA,objB;
    };
    
	rwsim::dynamics::BodyInfo 			defaultinfo;
  
	const std::string 				dwcfile;
        rw::kinematics::State 				rwstate;
        rw::models::WorkCell::Ptr 			wc;
        rwsim::dynamics::DynamicWorkCell::Ptr 		dwc;

        double 						defaultcolmargin;
        std::string 					defaultRestModel;
        std::string 					defaultContactModel;
        bool 						autoDisable;
        rwsim::dynamics::MaterialDataMap 		materialData;
        std::string 					defaultMaterial;
        rwsim::dynamics::ContactDataMap 		contactData;
        std::string 					defaultObjectType;

        rw::common::PropertyMap 			engineProps;
        rw::math::Vector3D<> 				_gravity;

        std::vector<FrictionDataTmp> 			fdatas;
        std::vector<ContactDataTmp> 			cdatas;

        std::vector<rw::kinematics::Frame*> 		deviceBases;
        std::string 					dir;
	
private:
	void readMaterialDataList(PTree& tree);
	void readFrictionDatas(PTree& tree, std::string first, std::string second);
	void readFrictionMap(PTree& tree);
	void readContactDatas(PTree& tree, std::string first, std::string second);
	void readContactMap(PTree& tree);
	void readContactDataList(PTree& tree);
	void readPDDeviceController(PTree& tree);
	void readPoseDeviceController(PTree& tree);
	rwsim::dynamics::RigidDevice::Ptr readRigidDevice(PTree& tree);
	rwsim::dynamics::RigidBody::Ptr readRigidBody(PTree& tree, const std::string& prefix);
	rwsim::dynamics::RigidBody::Ptr readRigidBody(PTree& tree);
	rwsim::dynamics::FixedBody::Ptr readFixedBase(PTree& tree, rw::models::JointDevice *dev);
	rwsim::dynamics::Body::Ptr readRefBody(PTree& tree);
	rw::kinematics::Frame* getFrameFromAttr(PTree& tree, const std::string& attr);
	rw::kinematics::Frame* getFrameFromAttr(PTree& tree, const std::string& attr, const std::string& prefix);
	rw::models::Object::Ptr getObjectFromAttr(PTree& tree, const std::string& attr);
	rw::models::Object::Ptr getObjectFromAttr(PTree& tree, const std::string& attr, const std::string& prefix);
	rw::models::JointDevice::Ptr getJointDeviceFromAttr(PTree& tree);
	rw::models::Device::Ptr getDeviceFromAttr(PTree& tree);
	rwsim::dynamics::DynamicDevice::Ptr findDynamicDevice(rw::models::Device* dev);
	std::pair<rwsim::dynamics::BodyInfo, rw::models::Object::Ptr> readRigidJoint(PTree& tree, rw::models::JointDevice *device);
	rwsim::dynamics::KinematicBody::Ptr readKinematicBody(PTree& tree, const std::string& frameAttr);
	rwsim::dynamics::KinematicBody::Ptr readKinematicBody(PTree& tree,const std::string& frameAttr,const std::string& prefix);
	std::pair<rwsim::dynamics::BodyInfo, rw::models::Object::Ptr> readLink(PTree& tree, rw::models::JointDevice *device, bool kinematic=false);
        rwlibs::control::JointController::ControlMode readControlMode(PTree& tree, const std::string& tname );
	void readProperties(PTree& tree, rw::common::PropertyMap& map);
	rw::math::Q getValueAsQ(const std::string& strlist );
	rw::math::InertiaMatrix<> readInertia(PTree& tree);
	rw::math::Vector3D<> readVector3D(PTree& tree);
	rw::math::Q readQ(PTree& tree);
	bool readBool(PTree& tree);
	std::vector<double> readArray(PTree& tree);
	std::pair<bool, double> toDouble(const std::string& str);
	std::string quote(const std::string& str);
  
};
}
}













#endif // DYNAMICWORKCELL_BUILDER_HPP_