/*
 * Pose_estimation.cpp
 *
 *  Created on: Sep 27, 2013
 *      Author: Thomas Sølund
 */
// ROS
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Transform.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl/PCLPointCloud2.h>

// Boost
#include <boost/algorithm/string.hpp>
#include <boost/bimap.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/shared_ptr.hpp>

//Boost uuid
#include <boost/uuid/uuid.hpp>            // uuid class
#include <boost/uuid/uuid_generators.hpp> // generators
#include <boost/uuid/uuid_io.hpp>         // streaming operators etc.
#include <boost/lexical_cast.hpp>

// STL
#include <map>

#include <pose_estimation_covis/CADModel.h>

//#include <robot_srvs/ObjectDetectionSegmented.h>
//#include <robot_srvs/poseEstimation.h>
//#include <robot_msgs/ResultPose.h>

// Services
#include <pose_estimation_covis/prepareEstimation.h>
#include <pose_estimation_covis/estimate.h>

// CoViS
#include <PoseEstimation/PoseEstimation.h>
using namespace PoseEstimation;

// Own
typedef pose_estimation_covis::estimate MsgEstimate;
typedef MsgEstimate::Request ReqEstimate;
typedef MsgEstimate::Response RespEstimate;

typedef pose_estimation_covis::prepareEstimation MsgPrepare;
typedef MsgPrepare::Request ReqPrepare;
typedef MsgPrepare::Response RespPrepare;


//typedef DescRGBN DescT;
ros::Publisher pubvis;
boost::mutex mpubvis;
tf::TransformBroadcaster* tb;

// Lock
boost::mutex mutex;

// PoseEstimation parameters
typedef DescRGBN DescT;
typedef DescRGBN::Vec DescVecT;

// Recognition object
typename Recognition<DescT>::Ptr rec;

// Utility
AlignmentUtil au;
DescriptorUtil du;

// Map for objects, uuid --> model object
std::map<boost::uuids::uuid,boost::shared_ptr<perception::CADModel> > indexmap;

// Map for objects, uuid --> model object
std::map<int, boost::uuids::uuid> indexUUid;

// Object point clouds, both as message type and as descriptor type
const int numobj = 7;
sensor_msgs::PointCloud2 msgobj[numobj];
DescT::Vec objects[numobj];

// Recognition parameters
bool table = true; // True for table removal
bool planar = true; // True for planar point removal
bool outliers = true; // True for outlier removal
bool reversee = false; // True for reverse feature matching
bool gravity = false; // True for gravity assumption (requires table to be true)
float cothres = 0.2; // Upper coplanarity fraction limit for detection pruning
float far = 1500.0f; // Background distance threshold
float leaf = 5.0f; // Resolution
float radius = 25.0f; // Feature radius
float threshold = 10.0f; // Inlier threshold
float fraction = 0.1f; // Required inlier fraction
int corrnum = 2; // Number of feature correspondences to use
bool verbose = false;

std::string global_topic;
std::string local_topic;

// Extra service for segmentation-based detection
//typedef robot_srvs::ObjectDetectionSegmented SegmentedMsgT;

using namespace perception;

//Function prototypes
bool prepareEstimation(ReqPrepare &req, RespPrepare &res);
bool estimate(ReqEstimate& req, RespEstimate& resp);
bool local(const boost::uuids::uuid& uid, const KMatrix<>& pose_guess, const std::string& mv_topic, RespEstimate& resp);
bool visualization(const Detection::Vec&,
      const sensor_msgs::PointCloud2&,
      const KMatrix<>& guess = KMatrix<>());

/*
 * Main entry point
 */
int main(int argc, char **argv) {
   // Initialize node
   const std::string name = "pose_estimation_pcl";
   ros::init(argc, argv, name);
   ros::NodeHandle n("~");

   //Instantiate tf object
   tb = new tf::TransformBroadcaster;


   //Get global point cloud topics from the parameter server (single view topic)
   if(ros::param::has("~global_topic"))
	  ros::param::get("~global_topic", global_topic);
   if(ros::param::has("~local_topic"))
	  ros::param::get("~local_topic", local_topic);

   // Get optional recognition parameters
     if(ros::param::has("~table"))
        ros::param::get("~table", table);
     if(ros::param::has("~planar"))
        ros::param::get("~planar", planar);
     if(ros::param::has("~outliers"))
        ros::param::get("~outliers", outliers);
     if(ros::param::has("~reverse"))
        ros::param::get("~reverse", reversee);
     if(ros::param::has("~far"))
        ros::param::get("~far", far);
     if(ros::param::has("~gravity"))
        ros::param::get("~gravity", gravity);
     if(ros::param::has("~verbose"))
        ros::param::get("~verbose", verbose);

     double dtmp;
     if(ros::param::has("~cothres")) {
        ros::param::get("~cothres", dtmp);
        cothres = float(dtmp);
     }
     if(ros::param::has("~leaf")) {
        ros::param::get("~leaf", dtmp);
        leaf = float(dtmp);
     }
     if(ros::param::has("~radius")) {
        ros::param::get("~radius", dtmp);
        radius = float(dtmp);
     }
     if(ros::param::has("~threshold")) {
        ros::param::get("~threshold", dtmp);
        threshold = float(dtmp);
     }
     if(ros::param::has("~fraction")) {
        ros::param::get("~fraction", dtmp);
        fraction = float(dtmp);
     }
     if(ros::param::has("~corrnum")) {
        ros::param::get("~corrnum", corrnum);
     }

   /*
    * Setup services
    */
    ROS_INFO_STREAM("Advertising service \"" << n.getNamespace() << "/estimate\"...");
    ros::ServiceServer servEstimate = n.advertiseService<ReqEstimate, RespEstimate>("estimate", estimate);

    ROS_INFO_STREAM("Advertising service \"" << n.getNamespace() << "/prepareEstimation\"...");
    ros::ServiceServer servPrepare = n.advertiseService<ReqPrepare, RespPrepare>("prepareEstimation", prepareEstimation);

    ROS_INFO_STREAM("Advertising service \"" << n.getNamespace() << "/segmented\"...");
    //ros::ServiceServer servsegmented = n.advertiseService<SegmentedMsgT::Request, SegmentedMsgT::Response>("segmented", segmented);

    //Instantiate Recognition Object
    rec.reset(new RecognitionVoting<DescT>(leaf, radius, threshold, fraction, table, planar, outliers, far));

    rec->setCoplanarityFraction(cothres);
    rec->setReverse(reversee);
    rec->setCorrNum(corrnum);
    rec->setGravity(gravity);
    rec->setVerbose(verbose);

    /*
    * Setup output visualization topic
    */
   const std::string topic = "result_rgbcloud";
   ROS_DEBUG_STREAM("Creating publisher \"" << n.getNamespace() << "/" << topic << "\"...");
   pubvis = n.advertise<sensor_msgs::PointCloud2>(topic, 50);

   /*
    * Spin
    */
   ros::spin();

   delete tb;

   return 0;
}

bool prepareEstimation(ReqPrepare &req, RespPrepare &res)
{
	ROS_DEBUG("Pose_Estimation_covis: Prepare estimation called!!");
	//Clear map before adding new models
	indexmap.clear();
	indexUUid.clear();

	if(req.model.size() <= 0)
	{
		ROS_WARN("Pose_Estimation_covis: No model provided in prepareEstimation!");
		return false;
	}

	for(size_t i = 0; i<= req.model.size()-1; i++){

		sensor_msgs::PointCloud2 m = req.model[i];
		std::string name = req.model_name[i];
		pcl::PointCloud<pcl::PointXYZRGB> pmodel;
		pcl::fromROSMsg(m, pmodel);

		// Create unique id
		boost::uuids::uuid uuid = boost::uuids::random_generator()();
		boost::shared_ptr<CADModel> model(new CADModel(name ,pmodel.makeShared(), uuid,i));
		indexmap.insert(std::make_pair(uuid, model));

		indexUUid.insert(std::make_pair((int)i,uuid));

		std::string id_str = boost::lexical_cast<std::string>(uuid);
		res.model_id.push_back(id_str);
	}

	ROS_INFO("Pose_Estimation_covis: %d models added!", (int)res.model_id.size());

	return true;
}

bool estimate(ReqEstimate& req, RespEstimate& resp) {
   ROS_DEBUG("Grabbing scene and performing segmentation + estimation...");

   //Scene Cloud
   sensor_msgs::PointCloud2::ConstPtr cloud;
   // If a scene cloud is given, use that, otherwise grab one from the stream
    if(req.scene.data.size() <= 0)
    {
       ROS_INFO("Try grabbing a point cloud from the stream...");
       // Get a point cloud
       if(!global_topic.empty()){
    	    cloud = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(global_topic, ros::Duration(5.0));
    	   if(!cloud) {
    		   ROS_WARN("Retrieval of point cloud failed!");
    		   return false;
    	   }
       }else
	   ROS_WARN("Failed to grab point cloud. No point cloud topic provided");
    }else
    {
    	ROS_INFO("Using scene cloud from service request...");
    	sensor_msgs::PointCloud2 p = req.scene;

    	cloud = boost::make_shared<sensor_msgs::PointCloud2>(p);
    }

    // Get the scene
    pcl::PointCloud<pcl::PointXYZRGB> s;
    pcl::fromROSMsg(*cloud, s);
    DescT::Vec scene = du.fromPCL<pcl::PointXYZRGB,DescT>(s);

    std::vector<DescVecT> objects;
    std::vector<std::string> labelvec;
    for(std::vector<std::string>::iterator it = req.model_id.begin(); it != req.model_id.end(); it++)
    {
    	std::string s_id = *it;
	std::cout << "received uuid: " << s_id << std::endl;
    	boost::uuids::uuid id = boost::lexical_cast<boost::uuids::uuid>(s_id);
    	boost::shared_ptr<CADModel> model = indexmap.find(id)->second;
    	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cad = model->getModel();
    	DescT::Vec obj = du.fromPCL<pcl::PointXYZRGB,DescT>(*cad);
    	objects.push_back(obj);
    	labelvec.push_back(model->getModelName());

    }

    // Load objects into the object database
    rec->loadObjects(objects,labelvec);
    std::cout << "object database size: " << (int)rec->getObjects().size() << std::endl;
    // Start Global pose estimation
    Detection::Vec detections = rec->recognize(scene);


  if(detections.empty()){
      ROS_ERROR("ERROR in detection step. Detection vector is empty!!");
      return false;
  }
  
    std::cout << "1"  << std::endl;
    // Report
    if(req.print)
    	print(detections);
     
    std::cout << "detections.size()" << (int)detections.size()  << std::endl;

   for(size_t i = 0; i<= detections.size()-1; i++)
   {
	   Detection& det = detections[i];
	  std::cout << "label: " << det.label  << std::endl;
std::cout << "3"  << std::endl;
std::cout << "cols: " << det.pose.cols() << "rows: " << det.pose.rows()  << std::endl;

	   // TODO: Scale [mm] --> [m]
	   det.pose(0,3) *= 0.001;
	   det.pose(1,3) *= 0.001;
	   det.pose(2,3) *= 0.001;

	   std::cout << "Pose\n:" << det.pose.toCvMat() << std::endl;
	   KMatrix<> homMat =  det.pose;

	   const boost::uuids::uuid uid = indexUUid.find(det.idx)->second;
	   std::string sid = boost::lexical_cast<std::string>(uid);

	   resp.detected_obj_id.push_back(sid);
	    std::cout << "uuid: " << sid << std::endl;
	  if(req.local_refinement) 
	  { //Run local refinement
	    local(uid, homMat, local_topic, resp); //req.multi_view_topic
	  }else
	  {
	   resp.error.push_back(det.rmse);
	   resp.inlier_fraction.push_back(det.inlierfrac);
	   resp.detected_obj_name.push_back(det.label);
	   std::cout << "detect lable: " << det.label << std::endl;
	   
	   geometry_msgs::Transform trans;
	   trans.translation.x = det.pose(0,3);
	   trans.translation.y = det.pose(1,3);
	   trans.translation.z = det.pose(2,3);
	   
	   KQuaternion Q = homMat.rotationQuaternion();
	  
	   trans.rotation.w =  Q[0];
	   trans.rotation.x =  Q[1];
	   trans.rotation.y =  Q[2];
	   trans.rotation.z =  Q[3];

	   resp.poses.push_back(trans);
	   
	    // Send to visualizer
           visualization(detections, *cloud);

	  /* ROS_INFO("=================Object: %s ===============",  det.label.c_str());
	   ROS_INFO("ERROR: %f", det.rmse);
	   ROS_INFO("INLIER_FRACTION: %f", det.inlierfrac);
	   ROS_INFO("PENALTY: %f", det.penalty);
	   ROS_INFO("IDX: %d", det.idx);
	   ROS_INFO("COFRAC: %f", det.params.find("cofrac")->second);
	  */
	  }
   }

     // Send to visualizer
     //visualization(detections, *cloud);

     ROS_INFO("Finished GLOBAL estimation!");

   return true;
}

bool local(const boost::uuids::uuid& uid, const KMatrix<>& pose_guess, const std::string& mv_topic, RespEstimate& resp) {
   // Lock members
   boost::mutex::scoped_lock lock(mutex);
   ROS_INFO("Running local refinement....");

   //Scene Cloud
   sensor_msgs::PointCloud2::ConstPtr cloud;
   //Grab one from the stream
   ROS_DEBUG("Try grabbing a point cloud from the stream...");
   // Get a point cloud
   if(!mv_topic.empty()){
      cloud = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(mv_topic, ros::Duration(5.0));
      if(!cloud) {
         ROS_WARN("Retrieval of point cloud failed in local refinement!");
       return false;
      }
   }else{
  	   ROS_WARN("Failed to grab point cloud. No point cloud topic provided");
  	   return false;
   }

   // Get the object
    boost::shared_ptr<CADModel> model = indexmap.find(uid)->second;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cad = model->getModel();
    DescT::Vec object = du.fromPCL<pcl::PointXYZRGB,DescT>(*cad);

    if(object.empty()) {
       ROS_ERROR("Empty object in local refinement!");
       return false;
    }

    // Get the scene
    pcl::PointCloud<pcl::PointXYZRGB> s;
    pcl::fromROSMsg(*cloud, s);
    DescT::Vec scene = du.fromPCL<pcl::PointXYZRGB,DescT>(s);

   // Preprocess
   scene = du.removeNaN<DescT>(scene);
   scene = du.removeFar<DescT>(scene, far);
   scene = du.downsample<DescT>(scene, leaf);

   // Report
   ROS_INFO_STREAM("Starting LOCAL estimation for \"" << model->getModelName() << "\"...");

   // Get the start guess
   KMatrix<> guess;
   guess.init(4, 4, pose_guess.data());
   if(guess.iseye())
      ROS_WARN("Pose guess seems to be identity!");

   // TODO: Scale [m] --> [mm] before alignment
   guess(0,3) *= 1000.0;
   guess(1,3) *= 1000.0;
   guess(2,3) *= 1000.0;

   // Run local alignment
   Alignment<DescT> align(object, scene, threshold, fraction);
   align.setVerbose(verbose);
   if( !align.icp(50, guess, true) ) {
	  ROS_ERROR_STREAM("Failed to do LOCAL estimation of object \"" << model->getModelName() << "\"!");
      //Do visualization of guess anyway --JPAPON
      Detection detection = KMatrix<>(); //Just make an empty detection
      detection.idx = model->getIDX();
      detection.label = model->getModelName();
      visualization(Detection::Vec(1,detection), *cloud, guess);
      return false;
   }

   // Get result
   Detection detection = align.getStrongHypotheses().front();
   detection.idx = model->getIDX();
   detection.label = model->getModelName();


   // TODO: Scale both guess and result back [mm] --> [m]
   guess(0,3) *= 0.001;
   guess(1,3) *= 0.001;
   guess(2,3) *= 0.001;
   detection.pose(0,3) *= 0.001;
   detection.pose(1,3) *= 0.001;
   detection.pose(2,3) *= 0.001;

   // Store response
   
   
   resp.error.push_back(detection.rmse);
   resp.inlier_fraction.push_back(detection.inlierfrac);
   resp.detected_obj_name.push_back(detection.label);
	
   geometry_msgs::Transform trans;
   trans.translation.x = detection.pose(0,3);
   trans.translation.y = detection.pose(1,3);
   trans.translation.z = detection.pose(2,3);
	   
   KQuaternion Q = detection.pose.rotationQuaternion();
	  
   trans.rotation.w =  Q[0];
   trans.rotation.x =  Q[1];
   trans.rotation.y =  Q[2];
   trans.rotation.z =  Q[3];

   resp.poses.push_back(trans);

   // Send to visualizer
   visualization(Detection::Vec(1,detection), *cloud, guess);

   // Report
   ROS_INFO_STREAM("Finished LOCAL estimation for \"" << model->getModelName() << "\"!");
   if(verbose) {
      ROS_INFO_STREAM("Pose guess:" << std::endl << guess << std::endl);
      ROS_INFO_STREAM("Refined pose:" << std::endl << detection.pose);
   }

  return true;
}

/*bool segmented(SegmentedMsgT::Request& req, SegmentedMsgT::Response& resp) {
   // Get the scene
   DescT::Vec segment = du.fromPCL<pcl::PointXYZRGB,DescT>(req.segment);
   if(segment.empty()) {
      ROS_ERROR("Empty segment point cloud!");
      return false;
   }

   if(req.labels_or.empty()) {
      ROS_WARN("No recognized objects specified!");
      return true;
   }

   // Report
   ROS_INFO("Starting SEGMENTATION-BASED estimation for the following objects:");
   for(unsigned int i = 0; i < req.labels_or.size(); ++i)
      ROS_INFO_STREAM("\t" << req.labels_or[i]);

   return true;
}
*/

// Utility functions for visualization
namespace {
inline pcl::PointCloud<pcl::PointXYZRGB>::Ptr transform(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& msg,
      const KMatrix<>& pose) {
   // Convert pose [mm] to Eigen [m]
   Eigen::Matrix4f posee;
   for(int r = 0; r < 4; ++r)
      for(int c = 0; c < 4; ++c)
         posee(r,c) = pose(r,c);

   // Convert point cloud to PCL type
   pcl::PointCloud<pcl::PointXYZRGB>::Ptr result(new pcl::PointCloud<pcl::PointXYZRGB>);
 //  pcl::fromROSMsg<pcl::PointXYZRGB>(msg, *result);

   // Transform
   pcl::transformPointCloud<pcl::PointXYZRGB>(*msg, *result, posee);

   return result;
}

inline void color(pcl::PointCloud<pcl::PointXYZRGB>& cloud,
      double r,
      double g,
      double b) {
   for(pcl::PointCloud<pcl::PointXYZRGB>::iterator it = cloud.begin(); it != cloud.end(); ++it) {
      it->r = 255*r;
      it->g = 255*g;
      it->b = 255*b;
   }
}
}
/// \endcond
bool visualization(const Detection::Vec& detections,
      const sensor_msgs::PointCloud2& scene,
      const KMatrix<>& guess) {

	// If a starting guess has been input, we are doing local estimation
   const bool local = guess.defined();

   // TODO: Clear all previous frames in tf
   const int maxnum = 25;
   tf::Transform Teye;
   Teye.setIdentity();
   for(int i = 0; i < maxnum; ++i) {
      std::ostringstream oss;
      oss << "/pose_estimation_covis_" << i;
      tb->sendTransform( tf::StampedTransform(Teye, ros::Time::now(), "/world", oss.str()) );
   }


   // Collect all detected objects for visualization
   pcl::PointCloud<pcl::PointXYZRGB> objall;
   for(unsigned int i = 0; i < detections.size(); ++i) {
      // Detection
      const Detection& di = detections[i];
      const boost::uuids::uuid uid = indexUUid.find(di.idx)->second;

      boost::shared_ptr<CADModel> model_obj = indexmap.find(uid)->second;

      // Transform point cloud message
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp ;

      // For local estimation, also visualize the guess in red //MOVED HERE JPAPON
      if (local) {
         tmp = ::transform(model_obj->getModel(), guess);
         ::color(*tmp, 1.0, 0.0, 0.0);
         objall += *tmp;
         if (!di.pose.defined ()) //If local and we didn't get a result, only display guess JPAPON
            continue;

      }

      tmp= ::transform(model_obj->getModel(), di.pose);

      // For local estimation, make it green, for global, random color
      if(local) {
         ::color(*tmp, 0.0, 1.0, 0.0);
      } else {
         double r, g, b;
         pcl::visualization::getRandomColors(r, g, b);
         ::color(*tmp, r, g, b);
      }

      // Append to output
      objall += *tmp;

      // For local estimation, also visualize the guess in red
      if(local) {
         tmp = ::transform(model_obj->getModel(), guess);
         ::color(*tmp, 1.0, 0.0, 0.0);
         objall += *tmp;
      }

      // TODO: Send the pose over tf, with fixed frame as world and ID set to idx
      std::ostringstream idxstr;
      idxstr << "/object_detection_" << di.idx;
      tf::Transform T;
      T.setOrigin( tf::Vector3(di.pose[3], di.pose[7], di.pose[11]) );
      tf::Matrix3x3 R;
      R.setValue(di.pose[0], di.pose[1], di.pose[2], di.pose[4], di.pose[5], di.pose[6], di.pose[8], di.pose[9], di.pose[10]);
      T.setBasis(R);
      tb->sendTransform( tf::StampedTransform(T, ros::Time::now(), "/world", idxstr.str()) );
   }

   // Convert to ROS type
   sensor_msgs::PointCloud2 objallmsg;
   pcl::toROSMsg(objall, objallmsg);
   objallmsg.header.frame_id = "object_detection_objects";

   // Copy input scene
   sensor_msgs::PointCloud2 scenemsg;
   
   pcl::PCLPointCloud2 in;
   pcl::PCLPointCloud2 out;
   pcl_conversions::toPCL(scene, in);
   
   pcl::copyPointCloud(in,out);
   pcl_conversions::fromPCL(out,scenemsg);
  
   scenemsg.header.frame_id = "object_detection_scene";

   // Publish to visualizer
   {
      boost::mutex::scoped_lock lock(mpubvis);
      pubvis.publish(objallmsg);
      pubvis.publish(scenemsg);
   }

   return true;
}


