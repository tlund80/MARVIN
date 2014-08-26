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


#include <one_shot_learning/motion_planner/planner.hpp>
#include <rw/math/Q.hpp>

namespace dti{
namespace one_shot_learning {
namespace motion_planner {
  
Planner::Planner(boost::shared_ptr<dti::grasp_planning::Workcell> workcell, boost::shared_ptr<PlannerConfiguration> Config) : _workcell(workcell), _config(Config)
{
  _samplerType = Planner::GAUSS;
  
   PlannerConfiguration *configuration = _config.get();
   
  if(_config->getPlannertype() == PlannerConfiguration::PRM)
  {
   
    PRMConfig* c = (PRMConfig*)configuration;
    _plannerType = Planner::PRM;
    
    std::cout << "Planner type is PRM!!" << std::endl;
    _motionPlanning_PRM.init(this, c);
  }else if (_config->getPlannertype() == PlannerConfiguration::RRT)
  {
     std::cout << "Planner type is RRT!!" << std::endl;
     RRTConfig* c = (RRTConfig*)configuration;
    _plannerType = Planner::RRT;
    _motionPlanning_RRT.init(this, c);
    
  }

}

Planner::~Planner()
{

}



bool Planner::plan(const rw::math::Q &qstart, const rw::math::Q &qgoal, rw::trajectory::Path<Q> &path)
{
	std::string colFrameStr;

	if(_workcell->inCollision(qstart, colFrameStr)) {
		RW_THROW("In collision!!");
	}

	if(_workcell->inCollision(qgoal, colFrameStr)) {
		RW_THROW("In collision!!");
	}

	// BUG: planner type cannot be switched dynamically
	switch(_plannerType)
	{
	  case Planner::PRM:
		  return _motionPlanning_PRM.plan(qstart, qgoal, path);
	  case Planner::RRT:
		  return _motionPlanning_RRT.plan(qstart, qgoal, path);
	  default:
		  RW_THROW("Unknown planner type!");
		  break;
	}
	return false;
}
void Planner::MotionPlanning_PRM::init(Planner *motionPlanning,PRMConfig* config)
{
	_motionPlanning = motionPlanning;
	if(config->getCollisionCheckingStrategy() == PlannerConfiguration::LAZY)_collisionCheckingStrategy = rwlibs::pathplanners::PRMPlanner::LAZY; 
	else if(config->getCollisionCheckingStrategy() == PlannerConfiguration::NODECHECK)_collisionCheckingStrategy = rwlibs::pathplanners::PRMPlanner::NODECHECK; 
	else if(config->getCollisionCheckingStrategy() == PlannerConfiguration::FULL)_collisionCheckingStrategy = rwlibs::pathplanners::PRMPlanner::FULL; 
	else std::cout << "Unknown Collision Cheking Strategy requested in Planner::MotionPlanning_PRM::init()" << std::endl;

	if(config->getNeighborSearchStrategy() == PlannerConfiguration::BRUTE_FORCE) _neighborSearchStrategy = rwlibs::pathplanners::PRMPlanner::BRUTE_FORCE;
	else if(config->getNeighborSearchStrategy() == PlannerConfiguration::PARTIAL_INDEX_TABLE) _neighborSearchStrategy = rwlibs::pathplanners::PRMPlanner::PARTIAL_INDEX_TABLE;
	else std::cout << "Unknown Neighbor search Strategy requested in Planner::MotionPlanning_PRM::init()" << std::endl;
 
	if(config->getShortestPathSearchStrategy() == PlannerConfiguration::A_STAR) _shortestPathSearchStrategy = rwlibs::pathplanners::PRMPlanner::A_STAR;
	else if(config->getShortestPathSearchStrategy() == PlannerConfiguration::DIJKSTRA) _shortestPathSearchStrategy = rwlibs::pathplanners::PRMPlanner::DIJKSTRA;
	else std::cout << "Unknown Shortest Path search Strategy requested in Planner::MotionPlanning_PRM::init()" << std::endl;
 
	_roadmapNodecount = config->getRoadmapNodecount();
	_maxtime = config->getMaxTime();
	_resolution = config->getResolution();
}
bool Planner::MotionPlanning_PRM::plan(const rw::math::Q &qstart, const rw::math::Q &qgoal, rw::trajectory::Path<Q> &path)
{
	using namespace rwlibs::pathplanners;
	using namespace rw::trajectory;
	using namespace rw::pathplanning;

	std::cout << "PRM Planner called!" << std::endl;
	dti::grasp_planning::Workcell* workcell = _motionPlanning->_workcell.get();

	QConstraint::Ptr qConstraintPtr = QConstraint::make( workcell->getCollisionDetector(), workcell->getRobot(), workcell->getState());
	QSampler::Ptr qSamplerPtr = _motionPlanning->getSampler(qstart, qgoal);
	PRMPlanner planner(qConstraintPtr, qSamplerPtr, _resolution, *(workcell->getRobot()), workcell->getState());

	planner.setCollisionCheckingStrategy(_collisionCheckingStrategy);
	planner.setNeighSearchStrategy(_neighborSearchStrategy);
	planner.setShortestPathSearchStrategy(_shortestPathSearchStrategy);
	planner.buildRoadmap(_roadmapNodecount);

	return planner.query(qstart, qgoal, path, _maxtime);
	
}

void Planner::MotionPlanning_RRT::init(Planner *motionPlanning,RRTConfig* config)
{
  _motionPlanning = motionPlanning;
  _extend = config->getExtend();
  _resolution = config->getResolution();
  std::cout << "_extend: " << _extend << std::endl;
  std::cout << "_resolution: " << _resolution << std::endl;
}

bool Planner::MotionPlanning_RRT::plan(const rw::math::Q &qstart, const rw::math::Q &qgoal, rw::trajectory::Path<rw::math::Q> &path)
{
        using namespace rwlibs::pathplanners;
	using namespace rw::trajectory;
	using namespace rw::pathplanning;
	using namespace rw::math;
	
	std::cout << "RRT Planner called!" << std::endl;
	dti::grasp_planning::Workcell *workcell = _motionPlanning->_workcell.get();
	EuclideanMetric<Q>::Ptr metricPtr(new EuclideanMetric<Q>());
	QConstraint::Ptr qConstraintPtr = QConstraint::make(workcell->getCollisionDetector(), workcell->getRobot(), workcell->getState());
	QEdgeConstraint::Ptr qEdgeConstraintPtr = QEdgeConstraint::make(qConstraintPtr, metricPtr, _resolution);
	PlannerConstraint plannerConstraint(qConstraintPtr, qEdgeConstraintPtr);
	QSampler::Ptr qSamplerPtr = _motionPlanning->getSampler(qstart, qgoal);
	QToQPlanner::Ptr planner = RRTPlanner::makeQToQPlanner(plannerConstraint, qSamplerPtr, metricPtr, _extend, RRTPlanner::RRTConnect);

	return planner->query(qstart, qgoal, path);
}

rw::pathplanning::QSampler::Ptr Planner::getSampler(const Q &qstart, const Q &qgoal) const
{
	switch(_samplerType)
	{
	case Planner::GAUSS:
		return rw::common::ownedPtr(new Planner::GaussSampler(qstart, qgoal, 1.0));
	case Planner::UNIFORM:
		return rw::pathplanning::QSampler::makeUniform(_workcell->getRobot());
	}

	return rw::pathplanning::QSampler::makeUniform(_workcell->getRobot());
}


} /* namespace next_best_view_planner */
}
}