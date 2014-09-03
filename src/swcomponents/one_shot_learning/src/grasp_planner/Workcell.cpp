/*
    <one line to give the program's name and a brief idea of what it does.>
    Copyright (C) 2014  <copyright holder> <email>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License along
    with this program; if not, write to the Free Software Foundation, Inc.,
    51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
*/


#include <one_shot_learning/grasp_planner/Workcell.hpp>

#include <string>

#include <rw/geometry/Geometry.hpp>
#include <rw/geometry/GeometryUtil.hpp>
#include <rw/invkin/IKMetaSolver.hpp>
#include <rw/invkin/JacobianIKSolver.hpp>
#include <rw/invkin/JacobianIKSolverM.hpp>
#include <rw/kinematics/MovableFrame.hpp>
#include <rw/kinematics/Kinematics.hpp>
#include <rw/loaders/WorkCellLoader.hpp>
#include <rw/models/TreeDevice.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <rw/trajectory.hpp>
//#include <RobWork/RobWorkStudio/src/RobWorkStudio.hpp>
namespace dti{
namespace grasp_planning
{

Workcell::Workcell()
{

}

Workcell::~Workcell()
{

}

void Workcell::init_workcell(std::string &workcellFilename, std::string &deviceName, std::string planning_frame)
{
  using namespace rw::proximity;
  using namespace rwlibs::proximitystrategies;

  if(workcellFilename.length() > 0)
  _workcell = rw::loaders::WorkCellFactory::load(workcellFilename); 
  _robot = _workcell->findDevice(deviceName);
  _robot_name = deviceName;

  if(!_robot.get())
    RW_THROW("No such device: " << deviceName);
	
  std::cout << "DOF=" << _robot->getDOF() << std::endl;
  
  _tcpFrame = _workcell->findFrame(planning_frame);
  if(!_tcpFrame) _tcpFrame = _robot->getEnd();
  
   std::cout << "TCPFrame: " << _tcpFrame->getName() << std::endl; 
  
  _collisionDetector = CollisionDetector::Ptr(new CollisionDetector(_workcell, ProximityStrategyFactory::makeCollisionStrategy("PQP")));
  _state = _workcell->getDefaultState();

}

void Workcell::init_workcell(rw::models::WorkCell::Ptr wc_ptr, std::string &deviceName, std::string planning_frame)
{
  using namespace rw::proximity;
  using namespace rwlibs::proximitystrategies;

  _workcell = wc_ptr;
  _robot = _workcell->findDevice(deviceName);
  _robot_name = deviceName;

  if(!_robot.get())
    RW_THROW("No such device: " << deviceName);
	
  std::cout << "DOF=" << _robot->getDOF() << std::endl;
  
  _tcpFrame = _workcell->findFrame(planning_frame);
  if(!_tcpFrame) _tcpFrame = _robot->getEnd();
  
  std::cout << "TCPFrame: " << _tcpFrame->getName() << std::endl; 
  
  _collisionDetector = CollisionDetector::Ptr(new CollisionDetector(_workcell, ProximityStrategyFactory::makeCollisionStrategy("PQP")));
  _state = _workcell->getDefaultState();

}

bool Workcell::createEmptyWorkcell()
{
 
  using namespace rw::models;
  using namespace rw::geometry;
  _workcell = WorkCell::Ptr(new WorkCell("empty_scene"));  
  return true;
}

void Workcell::addModel(std::string& model_name, std::string frame_name, rw::math::Transform3D<double> transformation)//rw::kinematics::Frame::Ptr object_frame)
{
  using namespace rw::models;
  using namespace rw::loaders;
  using namespace rw::graphics;
  using namespace rwsim::dynamics;
  using namespace rw::common;
  using namespace rw::geometry;
  using namespace rw::kinematics;
 
  MovableFrame::Ptr _frame = MovableFrame::Ptr(new MovableFrame(frame_name));
  _workcell->addFrame(_frame.get());
  _state = _workcell->getDefaultState();
  _frame->setTransform(transformation,_state);
  //Model3D::Object3D::Ptr m = Model3D::Object3D::Ptr(new Model3D::Object3D(model_name));
  //m->addTriangle();
  //Model3D::Ptr model =  Model3D::Ptr(new Model3D(model_name));  // Model3DFactory::loadModel(filepath, "object");
  
  //if(!model) RW_THROW("Could not load model from: " << filepath);
  //Geometry::Ptr geom = GeometryFactory::load(filepath);
  //geom->setName("object");
  //if(!geom) RW_THROW("Could not load geometry from: " << filepath);
  
 /* Object::Ptr obj = Object::Ptr(new Object(_frame.get()));
  obj->addModel(model);
  obj->addGeometry(geom);
  std::cout << "Name of the object: " << obj->getName()<< std::endl;

  _workcell->add(obj);
  
  //Add model to the dynamic workcell as a Body
  std::vector<Geometry::Ptr> geo; geo.push_back(geom);
  rw::math::InertiaMatrix<double> _mat = GeometryUtil::estimateInertia(0.6,geo);
   std::cout << "=============== Estimated Body Inertia ===============\n " << _mat << std::endl;
  Vector3D<double> _COG =  GeometryUtil::estimateCOG(geo);
    std::cout << "=============== Estimated Center of gravity ===============\n " << _COG << std::endl;
  BodyInfo _info; _info.mass = 0.6; _info.integratorType = "Euler"; _info.material = "Plastic"; _info.objectType = "hardObj"; _info.inertia = _mat; _info.masscenter = _COG; 
  _info.objects.push_back(obj);
  _info.print(std::cout);
  
   RigidBody::Ptr _body =  ownedPtr(new RigidBody(_info,obj));
  
 // FixedBody::Ptr _body = ownedPtr(new FixedBody(_info, obj));
  _body->setObject(obj);
  _dwc->addBody(_body);
 
  
  std::cout << "Number of bodies in the dynamic workcell => "<< int(_dwc->getBodies().size()) << std::endl;
  */
}


void Workcell::addModelFromFile(std::string& filepath, std::string frame_name, rw::math::Transform3D<double> transformation)//rw::kinematics::Frame::Ptr object_frame)
{
  using namespace rw::models;
  using namespace rw::loaders;
  using namespace rw::graphics;
  using namespace rwsim::dynamics;
  using namespace rw::common;
  using namespace rw::geometry;
  using namespace rw::kinematics;
 
  MovableFrame::Ptr _frame = MovableFrame::Ptr(new MovableFrame(frame_name));
  _workcell->addFrame(_frame.get());
  _state = _workcell->getDefaultState();
  _frame->setTransform(transformation,_state);
   
  Model3D::Ptr model = Model3DFactory::loadModel(filepath, "object");
  if(!model) RW_THROW("Could not load model from: " << filepath);
  Geometry::Ptr geom = GeometryFactory::load(filepath);
  geom->setName("object");
  if(!geom) RW_THROW("Could not load geometry from: " << filepath);
  
  Object::Ptr obj = Object::Ptr(new Object(_frame.get()));
  obj->addModel(model);
  obj->addGeometry(geom);
  std::cout << "Name of the object: " << obj->getName()<< std::endl;

  _workcell->add(obj);
  
  //Add model to the dynamic workcell as a Body
  std::vector<Geometry::Ptr> geo; geo.push_back(geom);
  rw::math::InertiaMatrix<double> _mat = GeometryUtil::estimateInertia(0.6,geo);
   std::cout << "=============== Estimated Body Inertia ===============\n " << _mat << std::endl;
  Vector3D<double> _COG =  GeometryUtil::estimateCOG(geo);
    std::cout << "=============== Estimated Center of gravity ===============\n " << _COG << std::endl;
  BodyInfo _info; _info.mass = 0.6; _info.integratorType = "Euler"; _info.material = "Plastic"; _info.objectType = "hardObj"; _info.inertia = _mat; _info.masscenter = _COG; 
  _info.objects.push_back(obj);
  _info.print(std::cout);
  
   RigidBody::Ptr _body =  ownedPtr(new RigidBody(_info,obj));
  
 // FixedBody::Ptr _body = ownedPtr(new FixedBody(_info, obj));
  _body->setObject(obj);
  _dwc->addBody(_body);
 
  
  std::cout << "Number of bodies in the dynamic workcell => "<< int(_dwc->getBodies().size()) << std::endl;
}

void Workcell::addBox(rw::kinematics::Frame::Ptr frame, float dx, float dy, float dz)
{
  using namespace rw::models;
  using namespace rw::graphics;
  using namespace rw::geometry;
  
  _workcell->addFrame(frame.get());
   
  Box b(dx,dy,dz);
  TriMesh::Ptr box = b.getTriMesh();
 
  Model3D::Material mat("black", 0.5,0.5,0.5);
  Model3D::Ptr model = Model3D::Ptr(new Model3D("box"));
  model->addTriMesh(mat,box.operator*());
 
  Object::Ptr obj = Object::Ptr(new Object(frame.get()));
  obj->addModel(model);
  _workcell->add(obj);
 
}

void Workcell::addCylinder(rw::kinematics::Frame::Ptr frame, float radius, float height)
{
  using namespace rw::models;
  using namespace rw::graphics;
  using namespace rw::geometry;
  
  _workcell->addFrame(frame.get());
   
  Cylinder c(radius,height);
  TriMesh::Ptr cy = c.getTriMesh();
 
  Model3D::Material mat("black", 0.5,0.5,0.5);
  Model3D::Ptr model = Model3D::Ptr(new Model3D("cylinder"));
  model->addTriMesh(mat,cy.operator*());
 
  Object::Ptr obj = Object::Ptr(new Object(frame.get()));
  obj->addModel(model);
  _workcell->add(obj);
 
}

void Workcell::addSphere(rw::kinematics::Frame::Ptr frame, float radius)
{
  using namespace rw::models;
  using namespace rw::graphics;
  using namespace rw::geometry;
  
  _workcell->addFrame(frame.get());
   
  Sphere s(radius);
  TriMesh::Ptr sp = s.getTriMesh();
 
  Model3D::Material mat("black", 0.5,0.5,0.5);
  Model3D::Ptr model = Model3D::Ptr(new Model3D("sphere"));
  model->addTriMesh(mat,sp.operator*());
 
  Object::Ptr obj = Object::Ptr(new Object(frame.get()));
  obj->addModel(model);
  _workcell->add(obj);
 
}

void Workcell::addGripper(GripperType type, std::string geometry_folder_path)
{
  using namespace rw::models;
  using namespace rwsim::loaders;
  using namespace rw::proximity;
  using namespace rwlibs::proximitystrategies;
  
  std::string wc_path;
  std::string dwc_path;
  wc_path.append(geometry_folder_path);
  dwc_path.append(geometry_folder_path);
  
  switch(type)
  {
    case SDH_PAR:
    {
        wc_path.append("/scene/empty_sdh_scene/empty_sdh_scene.wc.xml");
	dwc_path.append("/scene/empty_sdh_scene/empty_sdh_scene.dwc.xml");
        _workcell = rw::loaders::WorkCellFactory::load(wc_path); 
	_dwc = DynamicWorkCellLoader::load(dwc_path);
	
	_gripper = _workcell->findDevice("SchunkHand");
	if(!_gripper.get())
	  RW_THROW("No such gripper: " << "SchunkHand");
	
	_tcpFrame = _workcell->findFrame("SchunkHand.SDHTCP");
       if(!_tcpFrame) _tcpFrame = _gripper->getEnd();
	 
        _base = _workcell->findFrame<rw::kinematics::MovableFrame>("SchunkHand.Base");
	
	Q openQ(7,-1.571,-1.571,1.571, -1.048, 0.174, -1.048, 0.174);
       _state = _workcell->getDefaultState();
       _gripper->setQ(openQ, _state);   
      
       
    } 
      break;
      
    case SDH_PAR1:
    {
        wc_path.append("/scene/empty_sdh_scene/empty_sdh_scene.wc.xml");
	dwc_path.append("/scene/empty_sdh_scene/empty_sdh_scene.dwc.xml");
       _workcell = rw::loaders::WorkCellFactory::load(wc_path); 
       	_dwc = DynamicWorkCellLoader::load(dwc_path);
       
       _gripper = _workcell->findDevice("SchunkHand");
	if(!_gripper.get())
	  RW_THROW("No such gripper: " << "SchunkHand");
	
	_tcpFrame = _workcell->findFrame("SchunkHand.SDHTCP1");
       if(!_tcpFrame) _tcpFrame = _gripper->getEnd();
	 
        _base = _workcell->findFrame<rw::kinematics::MovableFrame>("SchunkHand.Base");
	
	Q openQ(7,-1.571,-1.571,1.571, -0.296, 0.240, -0.296, 0.240);
       _state = _workcell->getDefaultState();
       _gripper->setQ(openQ, _state);   
    } 
    break;
    case SDH_PAR2:
    {
        wc_path.append("/scene/empty_sdh_scene/empty_sdh_scene.wc.xml");
	dwc_path.append("/scene/empty_sdh_scene/empty_sdh_scene.dwc.xml");
       _workcell = rw::loaders::WorkCellFactory::load(wc_path);
       	_dwc = DynamicWorkCellLoader::load(dwc_path);
        
       _gripper = _workcell->findDevice("SchunkHand");
	if(!_gripper.get())
	  RW_THROW("No such gripper: " << "SchunkHand");
	
	_tcpFrame = _workcell->findFrame("SchunkHand.SDHTCP1");
       if(!_tcpFrame) _tcpFrame = _gripper->getEnd();
	 
        _base = _workcell->findFrame<rw::kinematics::MovableFrame>("SchunkHand.Base");
	
	Q openQ(7,-1.571,-1.571,1.571, -0.1, 0.1, -0.1, 0.1);
       _state = _workcell->getDefaultState();
       _gripper->setQ(openQ, _state);  
    } 
    break; 
    
    case SDH_PAR1_TABLE:
    {
        wc_path.append("/scene/empty_sdh_scene/empty_sdh_scene.wc.xml");
	dwc_path.append("/scene/empty_sdh_scene/empty_sdh_scene.dwc.xml");
       _workcell = rw::loaders::WorkCellFactory::load(wc_path); 
       	_dwc = DynamicWorkCellLoader::load(dwc_path);
        
       _gripper = _workcell->findDevice("SchunkHand");
	if(!_gripper.get())
	  RW_THROW("No such gripper: " << "SchunkHand");
	
	_tcpFrame = _workcell->findFrame("SchunkHand.SDHTCP1");
       if(!_tcpFrame) _tcpFrame = _gripper->getEnd();
	 
        _base = _workcell->findFrame<rw::kinematics::MovableFrame>("SchunkHand.Base");
	
	Q openQ(7,-1.571,-1.571,1.571, -0.296, 0.240, -0.296, 0.240);
       _state = _workcell->getDefaultState();
       _gripper->setQ(openQ, _state);  
    } 
    break; 
    
    case SDH_PAR2_TABLE:
    {
        wc_path.append("/scene/empty_sdh_scene/empty_sdh_scene.wc.xml");
	dwc_path.append("/scene/empty_sdh_scene/empty_sdh_scene.dwc.xml");
       _workcell = rw::loaders::WorkCellFactory::load(wc_path); 
       	_dwc = DynamicWorkCellLoader::load(dwc_path);
        
       _gripper = _workcell->findDevice("SchunkHand");
	if(!_gripper.get())
	  RW_THROW("No such gripper: " << "SchunkHand");
	
	_tcpFrame = _workcell->findFrame("SchunkHand.SDHTCP1");
       if(!_tcpFrame) _tcpFrame = _gripper->getEnd();
	 
        _base = _workcell->findFrame<rw::kinematics::MovableFrame>("SchunkHand.Base");
	
	Q openQ(7,-1.571,-1.571,1.571, -0.1, 0.1, -0.1, 0.1);
       _state = _workcell->getDefaultState();
       _gripper->setQ(openQ, _state);  
    } 
    break; 
    
    case SDH_BALL:
    {
        wc_path.append("/scene/empty_sdh_scene/empty_sdh_scene.wc.xml");
	dwc_path.append("/scene/empty_sdh_scene/empty_sdh_scene.dwc.xml");
       _workcell = rw::loaders::WorkCellFactory::load(wc_path); 
       	_dwc = DynamicWorkCellLoader::load(dwc_path);
        
       _gripper = _workcell->findDevice("SchunkHand");
	if(!_gripper.get())
	  RW_THROW("No such gripper: " << "SchunkHand");
	
	_tcpFrame = _workcell->findFrame("SchunkHand.SDHTCP");
       if(!_tcpFrame) _tcpFrame = _gripper->getEnd();
	 
        _base = _workcell->findFrame<rw::kinematics::MovableFrame>("SchunkHand.Base");
	
	Q openQ(7,-1.048, 0.174, 1.047 ,-1.048, 0.174, -1.048, 0.174);
       _state = _workcell->getDefaultState();
       _gripper->setQ(openQ, _state);  
    } 
    break; 
    
    case SDH_CYL:
    {
        wc_path.append("/scene/empty_sdh_scene/empty_sdh_scene.wc.xml");
	dwc_path.append("/scene/empty_sdh_scene/empty_sdh_scene.dwc.xml");
       _workcell = rw::loaders::WorkCellFactory::load(wc_path); 
       	_dwc = DynamicWorkCellLoader::load(dwc_path);
        
       _gripper = _workcell->findDevice("SchunkHand");
	if(!_gripper.get())
	  RW_THROW("No such gripper: " << "SchunkHand");
	
	_tcpFrame = _workcell->findFrame("SchunkHand.SDHTCP");
       if(!_tcpFrame) _tcpFrame = _gripper->getEnd();
	 
        _base = _workcell->findFrame<rw::kinematics::MovableFrame>("SchunkHand.Base");
	
	Q openQ(7, -1.048, 0.174, 0.0, -1.048, 0.174,-1.048, 0.174);
       _state = _workcell->getDefaultState();
       _gripper->setQ(openQ, _state);  
    } 
    break; 
    
    case PG70:
    {
       wc_path.append("/scene/empty_pg70_scene/empty_pg70_scene.wc.xml");
       dwc_path.append("/scene/empty_pg70_scene/empty_pg70_scene.dwc.xml");
      _workcell = rw::loaders::WorkCellFactory::load(wc_path); 
      _dwc = DynamicWorkCellLoader::load(dwc_path);
	
      _gripper = _workcell->findDevice("PG70");
	if(!_gripper.get())
	  RW_THROW("No such gripper: " << "PG70");
	
	_tcpFrame = _workcell->findFrame("PG70.TCPPG70");
       if(!_tcpFrame) _tcpFrame = _gripper->getEnd();
	 
        _base = _workcell->findFrame<rw::kinematics::MovableFrame>("PG70.Base");
	
	Q openQ(1, 0.034);
       _state = _workcell->getDefaultState();
       _gripper->setQ(openQ, _state);  
    
    } 
      break;

    case PG70_SMALL:
    {
      wc_path.append("/scene/empty_pg70_scene/empty_pg70_scene.wc.xml");
      dwc_path.append("/scene/empty_pg70_scene/empty_pg70_scene.dwc.xml");
      _workcell = rw::loaders::WorkCellFactory::load(wc_path); 
      _dwc = DynamicWorkCellLoader::load(dwc_path);
    } 
      break;
      
    case GS20:
    {
      RW_THROW("GS20 gripper Not implemented yet! "); 
    } 
      break;
      
    case GS20_WIDE:
    {
      RW_THROW("GS20_WIDE gripper Not implemented yet! "); 
    } 
      break;
      
      
      
      
      
    default:
      break;
  }
  
   _collisionDetector = CollisionDetector::Ptr(new CollisionDetector(_workcell, ProximityStrategyFactory::makeCollisionStrategy("PQP")));
  
}

bool Workcell::inCollision(const Q& q, std::string& colFrameStr)
{
	using namespace rw::proximity;
	
	CollisionDetector::QueryResult queryResult;
	_robot->setQ(q, _state);
	if(_collisionDetector->inCollision(_state, &queryResult))
	{
		std::stringstream sStr;
		rw::kinematics::FramePairSet::iterator it;
		for(it=queryResult.collidingFrames.begin(); it!=queryResult.collidingFrames.end(); it++) {
			//sStr << "\t" << it->first->getName() << " <--> " << it->second->getName() << "\n";
			sStr << "[" << it->first->getName() << ", " << it->second->getName() << "]";
		}
		colFrameStr = sStr.str();
		return true;
	}

	return false;
}

bool Workcell::solve_invkin(const Transform3D<> &trans, bool stopAtFirst, const Q &initQ, std::vector<Q> &solutions, bool doCollisionDetection, bool enable_joint6swop)
{  
	using namespace rw::proximity;
	using namespace rw::invkin;
   
	CollisionDetector::Ptr collisionDetector;
	
	if (doCollisionDetection) {
		collisionDetector = _collisionDetector;
	}
	else {
		collisionDetector = NULL;
	}
	
	if(stopAtFirst)
		_robot->setQ(initQ, _state);
	
	JacobianIKSolver solver(_robot, _tcpFrame, _state);
	IKMetaSolver metaSolver(&solver, _robot, collisionDetector);
	metaSolver.setCheckJointLimits(true);
	
	solutions = metaSolver.solve(trans, _state, 100, stopAtFirst);
	
	if(solutions.empty() && enable_joint6swop)
	{
	
	  Rotation3D<> rot(1,0,0,0,-1,0,0,0,-1);
	  Rotation3D<> R = trans.R() * rot;
	  Vector3D<> T = trans.P();
	  Transform3D<> _new_trans(T,R);
	  
	  solutions = metaSolver.solve(_new_trans, _state, 100, stopAtFirst);
	  
	}
	
	return !solutions.empty();
	
}

bool Workcell::solve_invkin(std::string tcpFrameName, const Transform3D<> &trans, bool stopAtFirst, const Q &initQ, std::vector<Q> &solutions, bool enable_joint6swop)
{
  	using namespace rw::invkin;
	using namespace rw::kinematics;
	if(stopAtFirst)
		_robot->setQ(initQ, _state);
	Frame *tcpFrame = _workcell->findFrame(tcpFrameName);
	if(!tcpFrame)
		RW_THROW("The frame '" << tcpFrameName << "' doesn't exist");
	JacobianIKSolver solver(_robot, tcpFrame, _state);
	IKMetaSolver metaSolver(&solver, _robot);//, _collisionDetector);
	metaSolver.setCheckJointLimits(true);
	solutions = metaSolver.solve(trans, _state, 100, stopAtFirst);
	
	if(solutions.empty() && enable_joint6swop)
	{
	
	  Rotation3D<> rot(1,0,0,0,-1,0,0,0,-1);
	  Rotation3D<> R = trans.R() * rot;
	  Vector3D<> T = trans.P();
	  Transform3D<> _new_trans(T,R);
	  
	  solutions = metaSolver.solve(_new_trans, _state, 100, stopAtFirst);
	  
	}
	
	return !solutions.empty();
	
}

bool Workcell::solve_invkin_multi(const std::vector<Transform3D<> > &trans, Q &q)
{
	using namespace rw::invkin;
  	using namespace rw::kinematics;
	using namespace rw::models;
	
        State state = _workcell->getDefaultState();
	TreeDevice *treeDev = static_cast<TreeDevice*>(_robot.get());
	JacobianIKSolverM solver(treeDev, state);
	std::vector<Q> solutions = solver.solve(trans, state);
	if(solutions.empty())
		return false;
	q = solutions[0];
	return true;
}

void Workcell::test_moveJ(const Q& startQ, const Q& endQ)
{
  	using namespace rw::proximity;
	using namespace rw::trajectory;
	
	double t_total=10.0;
	double dt=0.01;
	CollisionDetector::QueryResult queryResult;
	QLinearInterpolator qLinearInterpolator(startQ, endQ, t_total);
	for(double t=0.0; t<=t_total; t+=dt)
	{
		Q q = qLinearInterpolator.x(t);
		_robot->setQ(q, _state);
		if(_collisionDetector->inCollision(_state, &queryResult))
		{
			RW_THROW("test_moveJ: Collision detected! (" << q << ")");
		}
	}
}

void Workcell::test_moveL(const Transform3D<> &startTrans, const Transform3D<> &endTrans)
{
  	using namespace rw::proximity;
	using namespace rw::trajectory;
	
	double x_total = (endTrans.P() - startTrans.P()).norm2();
	double dx=0.01;	// dx = 1 cm
	CollisionDetector::QueryResult queryResult;
	CartesianLinearInterpolator cartLinearInterpolator(startTrans, endTrans, x_total);
	for(double x=0.0; x<=x_total; x+=dx)
	{
		const bool stopAtFirst = true;
		Q initQ(6, 0.0); // starting guess for inverse kinmatics calculation		
		std::vector<rw::math::Q> solutions;
				
		Transform3D<> trans = cartLinearInterpolator.x(x);
		if (!solve_invkin(trans, stopAtFirst, initQ, solutions, false, false))
			RW_THROW("test_moveL: No inv.kin. solutions (x=" << x << ")");

		_robot->setQ(solutions[0], _state);
	}
}

Transform3D< double > Workcell::getTcp(const Q& q)
{
	_robot->setQ(q, _state);
	return rw::kinematics::Kinematics::frameTframe(getRobot()->getBase(), _tcpFrame, _state);
}

}
}