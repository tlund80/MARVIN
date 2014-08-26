/*
 * CADModel.cpp
 *
 *  Created on: Oct 7, 2013
 *      Author: thomas
 */

#include <pose_estimation_covis/CADModel.h>

namespace perception
{
CADModel::CADModel(std::string name, pcl::PointCloud<pcl::PointXYZRGB>::Ptr model, boost::uuids::uuid id, int idx)
	: _name(name),
	  _model(model),
	  _id(id),
	  _idx(idx){
	// TODO Auto-generated constructor stub

}

CADModel::~CADModel() {
	// TODO Auto-generated destructor stub
}

};
