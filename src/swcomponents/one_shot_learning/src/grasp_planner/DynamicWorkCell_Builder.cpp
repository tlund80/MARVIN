
#include <one_shot_learning/grasp_planner/DynamicWorkCell_Builder.hpp>

#include <rw/loaders/rwxml/XML.hpp>
USE_ROBWORK_NAMESPACE
using namespace robwork;

using namespace std;
//using namespace rwsim::loaders;
using namespace rwsim::dynamics;
using namespace rwsim::sensor;
using namespace rwsim::control;

using namespace rw::loaders;
using namespace rw::math;
using namespace rw::common;
using namespace rw::sensor;
using namespace rw::geometry;
using namespace rwlibs::control;

using namespace rw::models;
using namespace rw::kinematics;

using namespace rwlibs::simulation;


namespace dti{
namespace grasp_planning{
  
DynamicWorkCell_Builder::DynamicWorkCell_Builder(rw::models::WorkCell::Ptr workcell) :
      wc(workcell),
      _gravity(Vector3D<>(0.0,0.0,-9.82)),
      defaultcolmargin(0.01),
      defaultRestModel("Newton"),
      defaultContactModel("Guendel"),
      autoDisable(true), 
      defaultMaterial("Plastic"),
      defaultObjectType("hardObj")
{
  rw::math::InertiaMatrix<double> _mat = rw::math::InertiaMatrix<double>::makeCuboidInertia(1.0,0.0,0.0,0.0);
  Vector3D<double> _COG(0.0,0.0,0.0);
  defaultinfo.mass = 1.0;
  defaultinfo.integratorType = "Euler";
  defaultinfo.inertia = _mat;
  defaultinfo.masscenter = _COG;
  defaultinfo.material = "Plastic";
  defaultinfo.objectType = "hardObj";

  engineProps.add<std::string>("StepMethod", "", "WorldStep");
   
  rwstate = wc->getDefaultState();
  dwc = ownedPtr( new DynamicWorkCell(wc));
  
  BOOST_FOREACH(Device::Ptr dev, wc->getDevices()){
            deviceBases.push_back(dev->getBase());
  }
  
}

DynamicWorkCell_Builder::~DynamicWorkCell_Builder()
{
  
}

rwsim::dynamics::DynamicWorkCell::Ptr DynamicWorkCell_Builder::getWorkcell(void)
{
   // add all friction data too the materialdatamap
   BOOST_FOREACH(FrictionDataTmp &fdata, fdatas){
     try{
       materialData.addFrictionData(fdata.matA, fdata.matB, fdata.data);
    } catch (...){
      
    }   
  }
        
  BOOST_FOREACH(ContactDataTmp &cdata, cdatas){
    contactData.addNewtonData(cdata.objA, cdata.objB, cdata.data);
    
  }
        
 // TODO: now check if all bodies has a correct material and objectId. If they has not then
 // we use the default

 BOOST_FOREACH(Body::Ptr body, dwc->getBodies() ){
   if( body->getInfo().material == ""){
     if(defaultMaterial=="")
       RW_THROW("No default material defined! Either define one "
                "or specify material for all objects..");
       body->getInfo().material = defaultMaterial;
   }
   if( body->getInfo().objectType == ""){
     if(defaultObjectType=="")
       RW_THROW("No default object type defined! Either define one "
                "or specify object types for all objects..");
       body->getInfo().objectType = defaultObjectType;  
  }
   
}
  
  std::cout << "Gravity: " << _gravity << std::endl;
   dwc->setGravity(_gravity);
   dwc->getMaterialData() = materialData;
   dwc->getContactData() = contactData;
   dwc->getEngineSettings() = engineProps;
  
  return dwc;
}

void DynamicWorkCell_Builder::addRigidbody(rw::models::Object::Ptr object){
  rwsim::dynamics::BodyInfo _defaultinfo;
  _defaultinfo = defaultinfo;
  addRigidbody(object,_defaultinfo, true);
}

void DynamicWorkCell_Builder::addRigidbody(rw::models::Object::Ptr object, rwsim::dynamics::BodyInfo info, bool estimateInertia){
   Object::Ptr obj = wc->findObject(object->getName());
   if( !obj ){
     // create one without any geometry
     MovableFrame* baseframe = wc->findFrame<MovableFrame>(object->getName());
     if(baseframe==NULL)
	RW_THROW("no frame with name " << object->getName());
        obj = ownedPtr(new Object(baseframe));
     wc->add( obj );
    }

   // check if frame is actually a moveable frame
   MovableFrame *mframe = dynamic_cast<MovableFrame*>( object->getBase() );
   if( !mframe )
       RW_THROW("Object "<< quote(object->getName())<< " is not a movable frame!");
   
   //Set body info for the body
   BodyInfo _info;
   _info.mass =  info.mass;
   _info.material = info.material;
   _info.integratorType = info.integratorType;
   _info.objectType = info.objectType;
   _info.objects.push_back(obj);
  
    std::vector<Geometry::Ptr> geoms;
    BOOST_FOREACH(Object::Ptr assobj, info.objects){
       BOOST_FOREACH(Geometry::Ptr g, assobj->getGeometry()){
         geoms.push_back(g);
      }
   }
	
  if(estimateInertia && obj->getGeometry().size()!=0){
    Transform3D<> ref(info.masscenter);
    info.inertia = GeometryUtil::estimateInertia(info.mass, geoms, mframe,rwstate, ref);
  }else if(estimateInertia && obj->getGeometry().size()==0) {
    RW_WARN("No geomtry present to generate Inertia from. Default masscenter and inertia is used.");
    info.masscenter = Vector3D<>(0,0,0);
    info.inertia = InertiaMatrix<>::makeSolidSphereInertia(info.mass, 0.0001);
  }else{
    RW_WARN("Default masscenter and inertia is used.");
    info.masscenter = Vector3D<>(0,0,0);
    info.inertia = InertiaMatrix<>::makeSolidSphereInertia(info.mass, 0.0001); 
  }
		
  Log::debugLog()<< "Creating rigid body" << std::endl;
  RigidBody::Ptr body = ownedPtr( new RigidBody(info, obj) );
  dwc->addBody( body );
  
}

void DynamicWorkCell_Builder::addFixedbody(rw::models::Object::Ptr object, rwsim::dynamics::BodyInfo info){
  
        FixedBody::Ptr body = ownedPtr( new FixedBody(info, object) );
        dwc->addBody( body );
}

/*bool DynamicWorkCell_Builder::readKinematicDevice(PTree& tree){
        Log::debugLog()<< "ReadKinematicBody" << std::endl;
        JointDevice::Ptr device = getJointDeviceFromAttr(tree);
        std::vector<double> maxForce;
        //std::vector<KinematicBody*> bodies;

        std::vector<std::pair<BodyInfo,Object::Ptr> > bodies;
        Body::Ptr base = NULL;
        int jIdx = 0;
        std::string framePrefix = device->getName() + ".";
        for (CI p = tree.begin(); p != tree.end(); ++p) {
            if( p->first == "KinematicJoint" ){
                //KinematicBody *body = readBodyInfo(p->second, "joint", framePrefix, state);
                //bodies.push_back( body );
                std::pair<BodyInfo,Object::Ptr> part = readLink(p->second, device.get());
                bodies.push_back( part );
            } else if( p->first == "Link" ){
                std::pair<BodyInfo,Object::Ptr> part = readLink(p->second, device.get());
                bodies.push_back( part );
			} else if( p->first == "FixedBase" ){
				base = readFixedBase(p->second, state, device.get());
			} else if( p->first == "KinematicBase" ){
				base = readKinematicBody(p->second, "frame", framePrefix, state);
            } else if( p->first == "RefBase" ){
                // base reference to a frame in a body in which it is attached
                base = readRefBody(p->second, state);
                //

			} else if(p->first !="<xmlcomment>" && p->first != "<xmlattr>"){
                RW_THROW("Unknown element");
            }
            jIdx++;
        }
        if( !base ){
        	RW_THROW("Parser error - KinematicDevice must define a base (FixedBase or KinematicBase)");
        }
        KinematicDevice::Ptr kdev = ownedPtr(new KinematicDevice(base, bodies, device));
        std::vector<Body::Ptr> links = kdev->getLinks();
        BOOST_FOREACH(Body::Ptr l, links){
        	state.dwc->addBody( l );
        }
        state.dwc->addDevice( kdev );
        return kdev;
    
  }

*/
  
typedef boost::property_tree::ptree PTree;
void DynamicWorkCell_Builder::loadDynamicMaterialBase(const std::string& filename)
{
  std::string file = IOUtil::getAbsoluteFileName(filename);
  try {
        PTree tree;
        boost::property_tree::read_xml(file, tree);
	
	PTree child = tree.get_child("IncludeData");
	for (CI p = child.begin(); p != child.end(); ++p) {
         if (p->first == "MaterialData") {
                readMaterialDataList(p->second);
	  }else if (p->first == "FrictionMap") {
                readFrictionMap(p->second);
          }else if (p->first == "ObjectTypeData") {
                readContactDataList(p->second);
          }else if (p->first == "ContactMap") {
                readContactMap(p->second);
          }
	}
     // rw::loaders::XML::printTree(tree, std::cout);
      
    } catch (const boost::property_tree::ptree_error& e) {
        // Convert from parse errors to RobWork errors.
        RW_THROW(e.what());
    }
  
}

void DynamicWorkCell_Builder::loadGripperDevice(const std::string& filename)
{
  std::string file = IOUtil::getAbsoluteFileName(filename);
  try {
        PTree tree;
        boost::property_tree::read_xml(file, tree);
	
	PTree child = tree.get_child("IncludeData");
	for (CI p = child.begin(); p != child.end(); ++p) {
         if (p->first == "RigidDevice") {
                readRigidDevice(p->second);
	  }///// THE DIFFERENT CONTROLLER MODELS  
           else if(p->first == "PDDeviceController") {
                readPDDeviceController(p->second);
            } else if (p->first == "PoseDeviceController") {
                readPoseDeviceController(p->second);
            } else if (p->first == "SerialDeviceController") {
	        RW_THROW("SerialDeviceController not yet implemented in load function!");
            	//readSerialDeviceController(p->second);
            } else if (p->first == "SpringJointController"){
	       RW_THROW("SpringJointController not yet implemented in load function!");
	    }
	}
     // rw::loaders::XML::printTree(tree, std::cout);
      
    } catch (const boost::property_tree::ptree_error& e) {
        // Convert from parse errors to RobWork errors.
        RW_THROW(e.what());
    }
  
}

JointController::ControlMode DynamicWorkCell_Builder::readControlMode(PTree& tree, const std::string& tname ){
        string controlType = tree.get<std::string>(tname);
        if(controlType=="Position") return JointController::POSITION;
        else if(controlType=="CntPosition") return JointController::CNT_POSITION;
        else if(controlType=="Velocity") return JointController::VELOCITY;
        else if(controlType=="Force") return JointController::FORCE;
        else if(controlType=="Current") return JointController::CURRENT;

        RW_THROW("Control type: \"" << controlType << "\" is not supported!");
        return JointController::POSITION;
}

void DynamicWorkCell_Builder::readPDDeviceController(PTree& tree){
        Log::debugLog()<< "ReadDeviceControllerData" << std::endl;
        std::string controllername = tree.get_child("<xmlattr>").get<std::string>("name");

        Device::Ptr dev = getDeviceFromAttr(tree);

        if(dev==NULL)
            RW_THROW("No valid is referenced by the PDDeviceController.");

        bool useSyncPD = readBool( tree.get_child("Sync") );
        JointController::ControlMode controlType = readControlMode( tree.get_child("<xmlattr>"), "type" );
        std::vector<double> params_tmp = readArray( tree.get_child("PDParams") );
        double dt = tree.get<double>("TimeStep");

        RW_ASSERT(params_tmp.size()>1);
        std::vector<PDParam> params;
        for(size_t i=0;i<params_tmp.size()/2;i++){
            params.push_back( PDParam(params_tmp[2*i],params_tmp[2*i+1]));
        }

        DynamicDevice::Ptr ddev = findDynamicDevice(dev.get());
        if(useSyncPD){

            RW_THROW("Not currently supported!");
            //SyncPDController *controller = new SyncPDController();
        } else {

            PDController::Ptr controller = ownedPtr( new PDController(controllername, ddev, controlType, params, dt) );
            dwc->addController( controller );
        }

    }

void DynamicWorkCell_Builder::readPoseDeviceController(PTree& tree){
		Log::debugLog()<< "ReadDeviceControllerData" << std::endl;
		std::string controllername = tree.get_child("<xmlattr>").get<std::string>("name");

		Device::Ptr dev = getDeviceFromAttr(tree);

		if(dev==NULL)
			RW_THROW("No valid is referenced by the PoseDeviceController.");

		//Frame* tcp = getFrameFromAttr(tree, state, "tcp");

		double dt = tree.get<double>("TimeStep");

		DynamicDevice::Ptr ddev = findDynamicDevice(dev.get());
		PoseController::Ptr controller = ownedPtr( new PoseController(controllername, ddev, wc->getDefaultState(), dt) );
		dwc->addController( controller );
		//state.controllers.push_back( controller );

}

void DynamicWorkCell_Builder::readMaterialDataList(PTree& tree){
  std::cout << "Material ReadMaterialList " << std::endl;
        Log::debugLog()<< "ReadMaterialList" << std::endl;
        for (CI p = tree.begin(); p != tree.end(); ++p) {
            if (p->first == "Material") {
                string id = p->second.get_child("<xmlattr>").get<std::string>("id");
                string desc = p->second.get("Description","");
                materialData.add(id, desc);
		std::cout << "Material data " << id << " : " << desc << std::endl;
            } else if(p->first == "Default"){
                string defMaterial = p->second.get_value<string>();
                defaultMaterial = defMaterial;
            } else if(p->first!="<xmlcomment>"){
               // RW_THROW("Unknown element");
            }
           
        }
        std::cout << "getMaxMatID: " << materialData.getMaxMatID() << std::endl;
  
}

void DynamicWorkCell_Builder::readFrictionDatas(PTree& tree, string first, string second){
        Log::debugLog()<< "ReadFrictionDatas1" << std::endl;
        FrictionDataTmp dataTmp;
        dataTmp.matA = first;
        dataTmp.matB = second;
        for (CI p = tree.begin(); p != tree.end(); ++p) {
            if (p->first == "FrictionData") {
                Log::debugLog()<< "FrictionData" << std::endl;
                std::string typestr = p->second.get_child("<xmlattr>").get<std::string>("type");
                dataTmp.data.type = Coulomb; //matMap.getDataID(typestr);
                // all elements of FrictionData must be param arrays
                for (CI d = p->second.begin(); d != p->second.end(); ++d) {
                    if( d->first != "<xmlattr>" && d->first != "<xmlcomment>"){
                        Q q = readQ(d->second);
                        dataTmp.data.parameters.push_back(std::make_pair(d->first,q));
                    }
                }
                fdatas.push_back( dataTmp );
		std::cout << "FrictionData" << std::endl;
            } else if(p->first !="<xmlcomment>" && p->first != "<xmlattr>"){
                RW_THROW("Unknown element");
            }
        }

}

void DynamicWorkCell_Builder::readFrictionMap(PTree& tree){
        Log::debugLog()<< "FrictionMap" << std::endl;
        for (CI p = tree.begin(); p != tree.end(); ++p) {
            if (p->first == "Pair") {
                string first = p->second.get_child("<xmlattr>").get<std::string>("first");
                string second = p->second.get_child("<xmlattr>").get<std::string>("second");
                readFrictionDatas(p->second, first, second);
            } else if( p->first!="<xmlcomment>" ){
              //  RW_THROW("Unknown element: \"" << p->first << "\"" );
            }
        }
}

void DynamicWorkCell_Builder::readContactDatas(PTree& tree, string first, string second){
        Log::debugLog()<< "ReadFrictionDatas" << std::endl;
        ContactDataTmp dataTmp;
        dataTmp.objA = first;
        dataTmp.objB = second;
        for (CI p = tree.begin(); p != tree.end(); ++p) {
            if (p->first == "ContactData") {
                std::string typestr = p->second.get_child("<xmlattr>").get<std::string>("type");
                // todo: switch on type. Now we assume newton
                double cr = p->second.get<double>("cr");
                dataTmp.data.cr = cr;
                cdatas.push_back( dataTmp );
		std::cout << "ContactData" << std::endl;
            } else if(p->first !="<xmlcomment>" && p->first != "<xmlattr>"){
                RW_THROW("Unknown element");
            }
        }
 }

void DynamicWorkCell_Builder::readContactMap(PTree& tree){
        Log::debugLog()<< "ReadMaterialList" << std::endl;
        Log::debugLog()<< "FrictionMap" << std::endl;
        for (CI p = tree.begin(); p != tree.end(); ++p) {
            if (p->first == "Pair") {
                string first = p->second.get_child("<xmlattr>").get<std::string>("first");
                string second = p->second.get_child("<xmlattr>").get<std::string>("second");
                readContactDatas(p->second, first, second);
            } else if(p->first!="<xmlcomment>"){
                RW_THROW("Unknown element");
            }
        }
}

void DynamicWorkCell_Builder::readContactDataList(PTree& tree){
   Log::debugLog() << "ReadMaterialList" << std::endl;
   for (CI p = tree.begin(); p != tree.end(); ++p) {
      if (p->first == "ObjectType") {
          string id = p->second.get_child("<xmlattr>").get<std::string>("id");
          string desc = p->second.get("Description","");
          contactData.add(id, desc);
	  std::cout << "ObjectType" << std::endl;
        } else if(p->first == "Default"){
          string defObjectType = p->second.get_value<string>();
          defaultObjectType = defObjectType;
        } else if(p->first!="<xmlcomment>"){
          RW_THROW("Unknown element");
        }
    }
}

RigidDevice::Ptr DynamicWorkCell_Builder::readRigidDevice(PTree& tree){
        Log::debugLog()<< "ReadRigidDevice" << std::endl;
        JointDevice::Ptr device = getJointDeviceFromAttr(tree);
        //Q maxForce(device->getDOF());

        // first we get the force limits of all joints/constraints
        std::map<std::string, double> maxForceMap;
        for (CI p = tree.begin(); p != tree.end(); ++p) {
            if (p->first == "ForceLimit") {
                string refjoint = device->getName() + "." + p->second.get_child("<xmlattr>").get<std::string>("joint");
                maxForceMap[refjoint] = p->second.get_value<double>();
            }
        }
        // we put them in an array so that they are sorted correctly
        std::vector<double> maxForce;
        BOOST_FOREACH(Joint* joint, device->getJoints()){
            if( maxForceMap.find( joint->getName() ) == maxForceMap.end() ){
                RW_THROW("A force limit for the joint \"" << joint->getName()
                         << "\" in device \"" << device->getName() << "\" has not been defined!" );
            }
            maxForce.push_back( maxForceMap[joint->getName()] );
        }

        // next we find the base and all links
        std::vector<std::pair<BodyInfo,Object::Ptr> > bodies;
        Body::Ptr base = NULL;
        int jIdx = 0;
        std::string framePrefix = device->getName() + ".";
        for (CI p = tree.begin(); p != tree.end(); ++p) {
            //std::cout << p->first << std::endl;
        	if( p->first == "FixedBase" ){
            	base = readFixedBase(p->second, device.get());
            } else if( p->first == "KinematicBase" ){
		base = readKinematicBody(p->second, "frame", framePrefix);
            } else if( p->first == "RigidBase" ){
            	base = readRigidBody(p->second, framePrefix);
            } else if( p->first == "RefBase" ){
            	// base reference to a frame in a body in which it is attached
                base = readRefBody(p->second);
            } else if( p->first == "Link" ){
                std::pair<BodyInfo,Object::Ptr> part = readLink(p->second, device.get());
                bodies.push_back( part );
            } else if( p->first == "RigidJoint" ){
                // this can be both a link or a constraint or both. Here for backwards compatibility!!!!
                std::pair<BodyInfo,Object::Ptr> part = readRigidJoint(p->second, device.get());
                if(part.second!=NULL){
                    bodies.push_back( part );
                }
            } else if( p->first == "ForceLimit" ){
                // we allready processed this one
            } else if(p->first !="<xmlcomment>" && p->first != "<xmlattr>"){
                RW_THROW("Unknown element" << StringUtil::quote(p->first) );
            }
            jIdx++;
        }
        if( !base ){
        	RW_THROW("Parser error - RigidDevice must define a base (FixedBase or KinematicBase)");
        }
        RW_ASSERT(base!=NULL);

        RigidDevice::Ptr rigiddev = ownedPtr(new RigidDevice(base, bodies, device));
        rigiddev->setMotorForceLimits( Q(maxForce) );
        std::vector<Body::Ptr> links = rigiddev->getLinks();
        BOOST_FOREACH(Body::Ptr l, links){
            RW_DEBUG("Adding body to dwc: " << l->getName());
        	dwc->addBody( l );
        }
        RW_DEBUG("Adding device to dwc: " << rigiddev->getName());
        dwc->addDevice(rigiddev);
        return rigiddev;
	
}

RigidBody::Ptr  DynamicWorkCell_Builder::readRigidBody(PTree& tree, const std::string& prefix){
        Log::debugLog()<< "ReadRigidBody" << std::endl;

        string refframeName = tree.get_child("<xmlattr>").get<std::string>("frame");
        Object::Ptr obj = wc->findObject(prefix+refframeName);
        if( !obj ){
            // create one without any geometry
            MovableFrame* baseframe = wc->findFrame<MovableFrame>(prefix+refframeName);
            if(baseframe==NULL)
                RW_THROW("no frame with name " << prefix+refframeName);
            obj = ownedPtr(new Object(baseframe));
            wc->add( obj );
        }

        //Object::Ptr obj = getObjectFromAttr(tree, state, "frame", prefix);
        // check if frame is actually a moveable frame
        MovableFrame *mframe = dynamic_cast<MovableFrame*>( obj->getBase() );
        if( !mframe )
            RW_THROW("Object "<< quote(obj->getName())<< " is not a movable frame!");
        BodyInfo info;

        info.mass = tree.get<double>("Mass");
        info.material = tree.get<string>("MaterialID", defaultMaterial);
        info.integratorType = tree.get<string>("Integrator");
        info.objectType = tree.get<string>("ObjectID", defaultObjectType );
        info.objects.push_back(obj);
        // check if the body has multiple objects associated
        for (CI p = tree.begin(); p != tree.end(); ++p) {
            if( p->first == "Associate" ){
                std::string objname = p->second.get_child("<xmlattr>").get<std::string>("object");
                Object::Ptr assobj = wc->findObject(prefix+objname);
                if(assobj==NULL)
                    assobj = wc->findObject(objname);
                if(assobj==NULL)
                    RW_THROW("Associated Object does not exist: \"" << objname << "\"");
                info.objects.push_back(assobj);
            }
        }

        std::vector<Geometry::Ptr> geoms;
        BOOST_FOREACH(Object::Ptr assobj, info.objects){
            BOOST_FOREACH(Geometry::Ptr g, assobj->getGeometry()){
                geoms.push_back(g);
            }
        }


        // time to load the geometry
        Log::debugLog()<< "load geom" << std::endl;
        //info.frames = GeometryUtil::getAnchoredFrames( *mframe, state.rwstate);
        //std::vector<Geometry::Ptr> geometry = loadGeometry(mframe, info.frames, state.rwstate);

        boost::optional<string> def = tree.get_optional<string>("EstimateInertia");
        if(!def){
            info.masscenter = readVector3D( tree.get_child("COG") );
            info.inertia = readInertia( tree.get_child("Inertia") );
        } else {
        	if(obj->getGeometry().size()!=0){
        	    if( tree.get_optional<string>("COG") ){
        	        // if COG specified then use it and calculate inertia

        	        info.masscenter = readVector3D( tree.get_child("COG") );
        	        Transform3D<> ref(info.masscenter);
        	        info.inertia = GeometryUtil::estimateInertia(info.mass, geoms, mframe,rwstate, ref);
        	    } else {
        	        boost::tie(info.masscenter,info.inertia) = GeometryUtil::estimateInertiaCOG(info.mass, obj->getGeometry(), mframe,rwstate);
        	    }
        	} else {
        		RW_WARN("No geomtry present to generate Inertia from. Default masscenter and inertia is used.");
        		info.masscenter = Vector3D<>(0,0,0);
        		info.inertia = InertiaMatrix<>::makeSolidSphereInertia(info.mass, 0.0001);
        	}
        }


        readProperties(tree, mframe->getPropertyMap());

        //info.print();
        Log::debugLog()<< "Creating rigid body" << std::endl;
        RigidBody::Ptr body = ownedPtr( new RigidBody(info, obj) );
        dwc->addBody( body );
        return body;
    }

RigidBody::Ptr DynamicWorkCell_Builder::readRigidBody(PTree& tree){
    	return readRigidBody(tree,"");
      
}

KinematicBody::Ptr DynamicWorkCell_Builder::readKinematicBody(PTree& tree,
    		const std::string& frameAttr,
    		const std::string& prefix){
        Log::debugLog()<< "ReadKinematicBody" << std::endl;
        Frame *frame = getFrameFromAttr(tree, frameAttr, prefix);

        RW_DEBUG("Trying to find object: \"" << frame->getName() << "\"");
        Object::Ptr obj = wc->findObject( frame->getName() );
        if(obj==NULL){
            // TODO: unfortunately the robwork kinematic loader does not fully support the
            // Object loading yet. So we need to create an object for this particular frame
            RW_DEBUG("Adding new object to state!");
            obj = ownedPtr( new Object(frame) );
            wc->add(obj);
        }

        //Object::Ptr obj = getObjectFromAttr(tree, state, frameAttr, prefix);
        RW_DEBUG("Reading movable frame!");
        MovableFrame *refframe = dynamic_cast<MovableFrame*>( obj->getBase() );
        if(refframe==NULL) RW_THROW("The body frame of a Kinematic body must be a movable frame type!");
        string materialId = tree.get<string>("MaterialID");
        BodyInfo info;
        info.material = tree.get<string>("MaterialID",defaultMaterial);
        info.objectType = tree.get<string>("ObjectID",defaultObjectType );

        //info.frames = DynamicUtil::getAnchoredChildFrames( refframe, state.rwstate, state.deviceBases);
        //std::vector<Geometry::Ptr> geoms = loadGeometry(refframe, info.frames, state.rwstate);
        TimerUtil::sleepMs(500);
        RW_DEBUG("Creating kinematic body!");

        KinematicBody::Ptr body = ownedPtr( new KinematicBody(info, obj) );
        RW_DEBUG("Adding kinematic body!");
        dwc->addBody( body );

        RW_DEBUG("Added kinematic body!");
        //info.print();
        return body;
    }

KinematicBody::Ptr DynamicWorkCell_Builder::readKinematicBody(PTree& tree,
    		const std::string& frameAttr){
    	return readKinematicBody(tree,frameAttr,"");
      
 }

FixedBody::Ptr DynamicWorkCell_Builder::readFixedBase(PTree& tree, JointDevice *dev){
        // Log::infoLog() << "ReadFixedBase" << std::endl;
        Object::Ptr obj = getObjectFromAttr(tree, "frame", dev->getName()+".");

        BodyInfo info;
        info.material = tree.get<string>("MaterialID", defaultMaterial );
        info.objectType = tree.get<string>("ObjectID", defaultObjectType );

        //info.frames = DynamicUtil::getAnchoredChildFrames( refframe, state.rwstate, state.deviceBases);
        //std::vector<Geometry::Ptr> geoms = loadGeometry(refframe, info.frames, state.rwstate);
        FixedBody::Ptr body = ownedPtr( new FixedBody(info, obj) );
        //Log::infoLog() << "ReadFixedBody end" << std::endl;
        dwc->addBody( body );
        return body;
}

Body::Ptr DynamicWorkCell_Builder::readRefBody(PTree& tree){
        string refbodyName = tree.get_child("<xmlattr>").get<std::string>("body");
        return dwc->findBody( refbodyName );
}
    
Frame* DynamicWorkCell_Builder::getFrameFromAttr(PTree& tree, const std::string& attr){
        Log::debugLog()<< "getFrameFromAttr" << std::endl;
        string refframeName = tree.get_child("<xmlattr>").get<std::string>(attr);
        Frame* frame = wc->findFrame(refframeName);
        if( !frame )
            RW_THROW("Frame " << quote(refframeName) << " does not exist in workcell!");
        return frame;
}

Frame* DynamicWorkCell_Builder::getFrameFromAttr(PTree& tree, const std::string& attr, const std::string& prefix){
        Log::debugLog()<< "getFrameFromAttr" << std::endl;
        string refframeName = tree.get_child("<xmlattr>").get<std::string>(attr);
        Frame* frame = wc->findFrame(prefix+refframeName);
        if( !frame )
            RW_THROW("Frame " << quote(refframeName) << " does not exist in workcell!");
        return frame;
}

Object::Ptr DynamicWorkCell_Builder::getObjectFromAttr(PTree& tree, const std::string& attr){
        string refframeName = tree.get_child("<xmlattr>").get<std::string>(attr);
        Object::Ptr obj = wc->findObject(refframeName);
        if( !obj )
            RW_THROW("Object " << quote(refframeName) << " does not exist in workcell!");
        return obj;
}

Object::Ptr DynamicWorkCell_Builder::getObjectFromAttr(PTree& tree, const std::string& attr, const std::string& prefix){
        Log::debugLog()<< "getFrameFromAttr" << std::endl;
        string refframeName = tree.get_child("<xmlattr>").get<std::string>(attr);
        Object::Ptr obj = wc->findObject(prefix+refframeName);
        if( !obj )
            RW_THROW("Object " << quote(refframeName) << " does not exist in workcell!");
        return obj;
}

JointDevice::Ptr DynamicWorkCell_Builder::getJointDeviceFromAttr(PTree& tree){
        Log::debugLog()<< "Device from attr" << std::endl;
        string deviceName = tree.get_child("<xmlattr>").get<std::string>("device");
        Device::Ptr device = wc->findDevice(deviceName).get();
        if( device==NULL )
            RW_THROW("Device " << quote(deviceName) << " does not exist in workcell!");
        JointDevice::Ptr jdev = device.cast<JointDevice>();
        if(jdev==NULL)
            RW_THROW("Device " << quote(deviceName) << " is not a JointDevice!");
        return jdev;
}

Device::Ptr DynamicWorkCell_Builder::getDeviceFromAttr(PTree& tree){
        Log::debugLog()<< "Device from attr" << std::endl;
        string deviceName = tree.get_child("<xmlattr>").get<std::string>("device");
        Device::Ptr device = wc->findDevice(deviceName);
        if( device==NULL )
            RW_THROW("Device " << quote(deviceName) << " does not exist in workcell!");
        return device;
}

DynamicDevice::Ptr DynamicWorkCell_Builder::findDynamicDevice(Device* dev){
    	return dwc->findDevice(dev->getName());
}
    
std::pair<BodyInfo, Object::Ptr> DynamicWorkCell_Builder::readRigidJoint(PTree& tree, JointDevice *device ){
        Log::debugLog()<< "ReadRigidJoint" << std::endl;
        //string refjointName = tree.get_child("<xmlattr>").get<std::string>("joint");
        //Object::Ptr obj = state.wc->findObject(device->getName()+string(".")+refjointName);

        Frame *frame = getFrameFromAttr(tree, "joint", device->getName()+".");
        Object::Ptr obj = wc->findObject( frame->getName() );
        if(obj==NULL){
            // TODO: unfortunately the robwork kinematic loader does not fully support the
            // Object loading yet. So we need to create an object for this particular frame
            obj = ownedPtr( new Object(frame) );
            wc->add(obj);
        }


        if( obj==NULL )
            return std::pair<BodyInfo, Object::Ptr>(BodyInfo(), NULL);

        BodyInfo info;
        info.mass = tree.get<double>("Mass");
        info.material = tree.get<string>("MaterialID", defaultMaterial);
        info.objectType = tree.get<string>("ObjectID", defaultObjectType );

        boost::optional<string> def = tree.get_optional<string>("EstimateInertia");
        if(!def){
            info.masscenter = readVector3D( tree.get_child("COG") );
            info.inertia = readInertia( tree.get_child("Inertia") );
        } else {
        	if(obj->getGeometry().size()!=0){
        		if( tree.get_optional<string>("COG") ){
        			// if COG specified then use it and calculate inertia

        			info.masscenter = readVector3D( tree.get_child("COG") );
        			Transform3D<> ref(info.masscenter);
        			info.inertia = GeometryUtil::estimateInertia(info.mass, obj->getGeometry(), obj->getBase(),rwstate, ref);
        		} else {
        			boost::tie(info.masscenter,info.inertia) = GeometryUtil::estimateInertiaCOG(info.mass, obj->getGeometry(), obj->getBase(),rwstate);
        		}
        	} else {
        		RW_THROW("No geometry present to generate Inertia from Object: \"" << obj->getName() << "\"");
        	}
        }
        return std::make_pair(info, obj);
    
  }

std::pair<BodyInfo, Object::Ptr> DynamicWorkCell_Builder::readLink(PTree& tree, JointDevice *device, bool kinematic){
        Log::debugLog()<< "ReadLink" << std::endl;

        string refframeName = tree.get_child("<xmlattr>").get<std::string>("object");
        Object::Ptr obj = wc->findObject(device->getName()+"."+refframeName);
        if( !obj ){
            // create one without any geometry
            Frame* baseframe = wc->findFrame(device->getName()+"."+refframeName);
            if(baseframe==NULL)
                RW_THROW("no frame with name " << device->getName()+"." + refframeName);
            obj = ownedPtr(new Object(baseframe));
            wc->add( obj );
        }

        BodyInfo info;
        info.mass = tree.get<double>("Mass");
        info.material = tree.get<string>("MaterialID", defaultMaterial);
        info.objectType = tree.get<string>("ObjectID", defaultObjectType );


        readProperties(tree, obj->getBase()->getPropertyMap());

        info.objects.push_back(obj);
        // check if the body has multiple objects associated
        for (CI p = tree.begin(); p != tree.end(); ++p) {
            if( p->first == "Associate" ){
                std::string objname = p->second.get_child("<xmlattr>").get<std::string>("object");
                Object::Ptr assobj = wc->findObject(device->getName()+string(".")+objname);
                if(assobj==NULL)
                    assobj = wc->findObject(objname);
                if(assobj==NULL)
                    RW_THROW("Associated Object does not exist: \"" << objname << "\"");
                info.objects.push_back(assobj);
            }
        }

        std::vector<Geometry::Ptr> geoms;
        BOOST_FOREACH(Object::Ptr assobj, info.objects){
            BOOST_FOREACH(Geometry::Ptr g, assobj->getGeometry()){
                geoms.push_back(g);
            }
        }

        boost::optional<string> def = tree.get_optional<string>("EstimateInertia");
        if(!def){
            info.masscenter = readVector3D( tree.get_child("COG") );
            info.inertia = readInertia( tree.get_child("Inertia") );
        } else {
            if(obj->getGeometry().size()!=0){
        		if( tree.get_optional<string>("COG") ){
        			// if COG specified then use it and calculate inertia

        			info.masscenter = readVector3D( tree.get_child("COG") );
        			Transform3D<> ref(info.masscenter);
        			info.inertia = GeometryUtil::estimateInertia(info.mass, geoms, obj->getBase(),rwstate, ref);
        		} else {
                	boost::tie(info.masscenter,info.inertia) = GeometryUtil::estimateInertiaCOG(info.mass, geoms, obj->getBase(),rwstate);
                }
            } else {
                RW_THROW("No geometry present to generate Inertia from Object: \"" << obj->getName() << "\"");
            }
        }
        return std::make_pair(info, obj);
}

void DynamicWorkCell_Builder::readProperties(PTree& tree, PropertyMap& map){
        for (CI p = tree.begin(); p != tree.end(); ++p) {
            if(p->first == "Property") {
                std::string name = p->second.get_child("<xmlattr>").get<std::string>("name");
                //std::string desc = p->second.get_child_optional("Description").get<std::string>("desc","");
                std::string type = p->second.get_child("<xmlattr>").get<std::string>("type","string");
                if( type=="string" ){
                    std::string value = p->second.get_value<std::string>();
                    map.add<std::string>(name, "", value);
                } else if( type=="int" ){
                    int value = p->second.get_value<int>();
                    map.add<int>(name, "", value);
                } else if( type=="float" ){
                    double value = p->second.get_value<double>();
                    map.add<double>(name, "", value);
        		} else if( type=="Q" ){
        			rw::math::Q value = getValueAsQ(p->second.get_value<std::string>());
        			map.add<rw::math::Q>(name, "", value);
                } else {
                    RW_THROW("DynamicWorkCell_Builder: Unknown engine property type: " << StringUtil::quote(type) );
                }

            }
        }
}
   
rw::math::Q DynamicWorkCell_Builder::getValueAsQ(const std::string& strlist ){
        std::istringstream buf(strlist);
        std::vector<double> values;
        std::string str;
        while( buf >> str ){
        	double val = boost::lexical_cast<double>(str);
            values.push_back(val);
        }

        return rw::math::Q(values.size(), &values[0]);
}

rw::math::InertiaMatrix<> DynamicWorkCell_Builder::readInertia(PTree& tree){
        Q q = readQ(tree);
        if(q.size()==3){
            return InertiaMatrix<>(q[0],q[1],q[2]);
        } else if( q.size()==9){
            return InertiaMatrix<>(q[0],q[1],q[2],q[3],q[4],q[5],q[6],q[7],q[8]);
        }
        RW_THROW("Inertia needs either 3 or 9 arguments, it got " << q.size() );
}

Vector3D<> DynamicWorkCell_Builder::readVector3D(PTree& tree){
        Log::debugLog()<< "ReadVector3D" << std::endl;
        Q q = readQ(tree);
        if(q.size()!=3)
            RW_THROW("Unexpected sequence of values, must be length 3");
        return Vector3D<>(q[0],q[1],q[2]);
}
    
Q DynamicWorkCell_Builder::readQ(PTree& tree){
        Log::debugLog()<< "ReadQ" << std::endl;
        std::vector<double> arr = readArray(tree);
        Q q(arr.size());
        for(size_t i=0;i<q.size();i++){
            q[i] = arr[i];
        }
        return q;
}

std::vector<double> DynamicWorkCell_Builder::readArray(PTree& tree){
        //RW_DEBUGS( "ReadArray: " << tree.get_value<string>() );
        istringstream buf(tree.get_value<string>());
        std::vector<double> values;

        std::string str;
        while( buf >> str ){
            const pair<bool, double> okNum = toDouble(str);
            if (!okNum.first)
                RW_THROW("Number expected. Got \"" << str << "\" ");
            values.push_back(okNum.second);
        }
        return values;
  
}

bool DynamicWorkCell_Builder::readBool(PTree& tree){
        Log::debugLog()<< "ReadBool" << std::endl;
        string str= tree.get_value<string>();
        string strup = StringUtil::toUpper(str);
        if(strup=="TRUE" || strup=="1" || strup=="ON"){
            return true;
        } else if(strup=="FALSE" || strup=="0" || strup=="OFF"){
            return false;
        }
        RW_THROW("The input \"" << str << "\" is not a valid boolean!");
        return false;
}
    
std::pair<bool, double> DynamicWorkCell_Builder::toDouble(const std::string& str){
        std::pair<bool, double> nothing(false, 0);
        istringstream buf(str);
        double x;
        buf >> x;
        if (!buf) return nothing;
        string rest;
        buf >> rest;
        if (buf) return nothing;
        else return make_pair(true, x);
}
 
std::string DynamicWorkCell_Builder::quote(const string& str) { return StringUtil::quote(str); }
}
}