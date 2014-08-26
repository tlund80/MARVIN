/*
 * CADModel.h
 *
 *  Created on: Oct 2, 2013
 *      Author: thomas
 */

#ifndef CADMODEL_H_
#define CADMODEL_H_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

//Boost uuid
#include <boost/uuid/uuid.hpp>            // uuid class
#include <boost/uuid/uuid_generators.hpp> // generators
#include <boost/uuid/uuid_io.hpp>         // streaming operators etc.
#include <boost/lexical_cast.hpp>

namespace perception {

class CADModel {
public:
	CADModel(std::string name,
		 pcl::PointCloud<pcl::PointXYZRGB>::Ptr model,
		 boost::uuids::uuid id,
		 int idx);
	virtual ~CADModel();

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr getModel(void){return _model;};
	std::string getModelName(void){return _name;};
	boost::uuids::uuid getModelID(void){return _id;};
	int getIDX(void){return _idx;};

private:

	//Model name
	std::string _name;

	// PCL Model
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr _model;

	//UUid
	boost::uuids::uuid _id;

	//idx object recognition method internal id, strating from 0
	int _idx;

};
} /* namespace perception */
#endif /* CADMODEL_H_ */
