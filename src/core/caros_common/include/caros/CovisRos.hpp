/* */

#include <Features/texlet2D.h>
#include <caros_common_msgs/ECVTexlet2D.h>
#include <caros_common_msgs/ECVTexlets2D.h>

#include <Features/lineSegment3D.h>
#include <caros_common_msgs/ECVLineSegment3D.h>
#include <caros_common_msgs/ECVLineSegments3D.h>

#include <Features/extendedLineSegment3D.h>
#include <caros_common_msgs/ECVExtendedLineSegment3D.h>
#include <caros_common_msgs/ECVExtendedLineSegments3D.h>

#include "Features/texlet3D.h"
#include <caros_common_msgs/ECVTexlet3D.h>
#include <caros_common_msgs/ECVTexlets3D.h>

#include <Utilities/KIndexArray.h>
#include <caros_common_msgs/ECVLinks.h>


#include <Mathematics/CameraCalibration.h>
#include <Mathematics/StereoCalibration.h>
#include <sensor_msgs/CameraInfo.h>
#include <caros_common_msgs/CameraCalibration.h>
#include <caros_common_msgs/StereoCalibration.h>

#include <HighLevelFeatures/contour3D.h>
#include <caros_common_msgs/ECVContour3D.h>
#include <caros_common_msgs/ECVContours3D.h>

#include <HighLevelFeatures/abstractSurfling.h>
#include <HighLevelFeatures/abstractSurface.h>
#include <HighLevelFeatures/surfling3D.h>
#include <HighLevelFeatures/surface3D.h>
#include <caros_common_msgs/ECVSurfling3D.h>
#include <caros_common_msgs/ECVSurflings3D.h>
#include <caros_common_msgs/ECVSurface3D.h>
#include <caros_common_msgs/ECVSurfaces3D.h>

#include <caros_common_msgs/ECVRelationsVector.h>
#include <caros_common_msgs/ECVRelationsMatrix.h>




class CovisRos {
   public:
      static texlet2D toCovis(const caros_common_msgs::ECVTexlet2D& texlet2D);
      static caros_common_msgs::ECVTexlet2D toRos(const texlet2D& texlet2D);


      static lineSegment3D toCovis(const caros_common_msgs::ECVLineSegment3D& ls3D);
      static caros_common_msgs::ECVLineSegment3D toRos(const lineSegment3D& ls3D);
      
      static extendedLineSegment3D toCovis(const caros_common_msgs::ECVExtendedLineSegment3D& els3D);
      static caros_common_msgs::ECVExtendedLineSegment3D toRos(const extendedLineSegment3D& els3D);


      static texlet3D toCovis(const caros_common_msgs::ECVTexlet3D& texlet3D);
      static caros_common_msgs::ECVTexlet3D toRos(const texlet3D& texlet3D);


      static KLinkArray toCovis(const caros_common_msgs::ECVLinks& lnks);
      static caros_common_msgs::ECVLinks toRos(const KLinkArray& lnks);

      static Calibration::CameraCalibration toCovis(const sensor_msgs::CameraInfo& cam, bool undistorted = false);
      static sensor_msgs::CameraInfo toRos(const Calibration::CameraCalibration& cam);

      static std::pair< sensor_msgs::CameraInfo, sensor_msgs::CameraInfo > toRos(const Calibration::StereoCalibration& cams, double rectificationScaling = -1);
      static Calibration::StereoCalibration toCovis(const sensor_msgs::CameraInfo leftCam, const sensor_msgs::CameraInfo rightCam, bool rectified = false, bool undistorted = false);

      static caros_common_msgs::CameraCalibration toOwnRos(const Calibration::CameraCalibration& cam);
      static Calibration::CameraCalibration toCovis(const caros_common_msgs::CameraCalibration& cam);

      static caros_common_msgs::StereoCalibration toOwnRos(const Calibration::StereoCalibration& cams);
      static Calibration::StereoCalibration toCovis(const caros_common_msgs::StereoCalibration& cams);

      static std::vector <lineSegment3D> toCovis(const caros_common_msgs::ECVLineSegments3D& lss3D);
      static caros_common_msgs::ECVLineSegments3D toRos(const std::vector<lineSegment3D> & lss3D);

      static std::vector <extendedLineSegment3D> toCovis(const caros_common_msgs::ECVExtendedLineSegments3D& elss3D);
      static caros_common_msgs::ECVExtendedLineSegments3D toRos(const std::vector<extendedLineSegment3D> & elss3D);

      static std::vector <texlet3D> toCovis(const caros_common_msgs::ECVTexlets3D& ts3D);
      static caros_common_msgs::ECVTexlets3D toRos(const std::vector<texlet3D> & ts3D);

      static std::vector <texlet2D> toCovis(const caros_common_msgs::ECVTexlets2D& ts2D);
      static caros_common_msgs::ECVTexlets2D toRos(const std::vector<texlet2D> & ts2D);

      static HighLevelFeatures::contour3D toCovis(const caros_common_msgs::ECVContour3D& c3D, std::vector <lineSegment3D> & lineSegments3D, const KMatrix<double>  *projectionMatrixP);
      static caros_common_msgs::ECVContour3D toRos( HighLevelFeatures::contour3D& c3D);

      static HighLevelFeatures::surfling3D toCovis(const caros_common_msgs::ECVSurfling3D& sl3D, std::vector <texlet3D> & texlets3D);
      //simpler version, without assigning texlets, used for creating surfaces, texlets info not needed
      static HighLevelFeatures::surfling3D toCovis(const caros_common_msgs::ECVSurfling3D& sl3D);
      static caros_common_msgs::ECVSurfling3D toRos( HighLevelFeatures::surfling3D& sl3D);
      
      static HighLevelFeatures::surface3D toCovis(const caros_common_msgs::ECVSurface3D& s3D, std::vector <HighLevelFeatures::abstractSurfling*> &surflings3D);
      static caros_common_msgs::ECVSurface3D toRos( HighLevelFeatures::surface3D& s3D);


      static std::vector <HighLevelFeatures::contour3D*> toCovis(const caros_common_msgs::ECVContours3D& cs3D, std::vector <lineSegment3D> & lineSegments3D, const KMatrix<double>  *projectionMatrixP = NULL, bool orderContours = false);
      static caros_common_msgs::ECVContours3D toRos( std::vector<HighLevelFeatures::abstractContour*> & cs3D);
      static caros_common_msgs::ECVContours3D toRos( std::vector<HighLevelFeatures::contour3D*> & cs3D);

      static std::vector <HighLevelFeatures::abstractSurfling*> toCovis(const caros_common_msgs::ECVSurflings3D& sls3D, std::vector <texlet3D> & texlets3D);
      //simpler version, without assigning texlets, used for creating surfaces, texlets info not needed
      static std::vector <HighLevelFeatures::abstractSurfling*> toCovis(const caros_common_msgs::ECVSurflings3D& sls3D);
      static caros_common_msgs::ECVSurflings3D toRos( std::vector<HighLevelFeatures::abstractSurfling*> & sls3D);
      
      static std::vector <HighLevelFeatures::surface3D*> toCovis(const caros_common_msgs::ECVSurfaces3D& ss3D, std::vector <HighLevelFeatures::abstractSurfling*> &surflings3D);
      static caros_common_msgs::ECVSurfaces3D toRos( std::vector<HighLevelFeatures::abstractSurface*> & ss3D);
      static caros_common_msgs::ECVSurfaces3D toRos( std::vector<HighLevelFeatures::surface3D*> & ss3D);

      static std::vector<std::vector<double> > toCovis(const caros_common_msgs::ECVRelationsMatrix& relationMatrix);
      static caros_common_msgs::ECVRelationsMatrix toRos(const std::vector<std::vector<double> >& relationMatrix);

};
