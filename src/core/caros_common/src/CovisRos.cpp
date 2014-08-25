/* */
#include <caros/CovisRos.hpp>
#include <Representation/Scene/KPrimitiveLink.h>
#include <image_geometry/pinhole_camera_model.h>
#include <image_geometry/stereo_camera_model.h>
#include <Mathematics/KMatrix.h>

/****************************************************************************************
* Texlet2D conversion functions
*
*****************************************************************************************/

texlet2D CovisRos::toCovis(const caros_common_msgs::ECVTexlet2D& tex2D) {
   texlet2D result;

   result.setConfidence(tex2D.confidence);

   result.setLength( tex2D.length);

   result.setPosition( KVector<double>(tex2D.position[0], tex2D.position[1]));

//    for (int i = 0; i < 9; i++) {
//       result.PositionCovariance()[i] = tex2D.positionCovariance[i];
//    }

   result.setConfIDim0(tex2D.intrinsicDimensionality[0]);
   result.setConfIDim1(tex2D.intrinsicDimensionality[1]);
   result.setConfIDim2(tex2D.intrinsicDimensionality[2]);

   result.setCorrespondingTexlet3D(tex2D.texlet3Dindex);

   result.setColorTriplet( KVector<double> ( tex2D.color[0],
                           tex2D.color[1],
                           tex2D.color[2]));

   return result;
}

caros_common_msgs::ECVTexlet2D CovisRos::toRos(const texlet2D& tex2D) {
   caros_common_msgs::ECVTexlet2D result;

   result.confidence = tex2D.conf();

   result.length = tex2D.length();

   result.position[0] = tex2D.X();
   result.position[1] = tex2D.Y();

//    for (int i = 0; i < 9; i++) {
//       result.positionCovariance[i] = tex2D.PositionCovariance()[i];
//    }

   result.intrinsicDimensionality[0] = tex2D.confIDim0();
   result.intrinsicDimensionality[1] = tex2D.confIDim1();
   result.intrinsicDimensionality[2] = tex2D.confIDim2();

   result.texlet3Dindex = tex2D.getCorrespondingTexlet3D();

   for (int i = 0; i < 3; i++) {
      result.color[i] = tex2D.colorTriplet()[i];
   }

   return result;
}



/****************************************************************************************
* LineSegment3D conversion functions
*
*****************************************************************************************/

lineSegment3D CovisRos::toCovis(const caros_common_msgs::ECVLineSegment3D& ls3D) {
   lineSegment3D result;

   result.setConfidence(ls3D.confidence);

   result.setLength( ls3D.length);

   result.setUID(ls3D.index);

   result.setPosition( KVector<double> ( ls3D.position[0],
                                         ls3D.position[1],
                                         ls3D.position[2]));

   for (int i = 0; i < 9; i++) {
      result.PositionCovariance()[i] = ls3D.positionCovariance[i];
   }

   result.setConfIDim0(ls3D.intrinsicDimensionality[0]);
   result.setConfIDim1(ls3D.intrinsicDimensionality[1]);
   result.setConfIDim2(ls3D.intrinsicDimensionality[2]);







   result.initSource2DLineSegments( ls3D.sourceIndices[0],
                                    ls3D.sourceIndices[1]);

   result.setPhiRadPsiRad( ls3D.orientation[0],
                           ls3D.orientation[1]);

   result.setOriConf( ls3D.orientationConfidence);

   for (int i = 0; i < 9; i++) {
      result.OrientationCovariance()[i] = ls3D.orientationCovariance[i];
   }

   result.setPhase( ls3D.phaseAngle);

   result.setPhaseConf( ls3D.phaseConfidence);

   result.PhaseVariance = ls3D.phaseVariance;

   for (int i = 0; i < 3; i++) {
      result.colorTripletLeft()[i] = ls3D.colorLeft[i];
   }
   result.setColConfL( ls3D.colorLeftConfidence);

   for (int i = 0; i < 3; i++) {
      result.colorTripletMiddle()[i] = ls3D.colorMiddle[i];
   }
   result.setColConfM( ls3D.colorMiddleConfidence);

   for (int i = 0; i < 3; i++) {
      result.colorTripletRight()[i] = ls3D.colorRight[i];
   }
   result.setColConfR( ls3D.colorRightConfidence);

   return result;
}

caros_common_msgs::ECVLineSegment3D CovisRos::toRos(const lineSegment3D& ls3D) {
   caros_common_msgs::ECVLineSegment3D result;

   result.confidence = ls3D.getConfidence();

   result.length = ls3D.getLength();

   result.index = ls3D.getUID();

   result.position[0] = ls3D.X();
   result.position[1] = ls3D.Y();
   result.position[2] = ls3D.Z();

   for (int i = 0; i < 9; i++) {
      result.positionCovariance[i] = ls3D.PositionCovariance()[i];
   }

   result.intrinsicDimensionality[0] = ls3D.getConfIDim0();
   result.intrinsicDimensionality[1] = ls3D.getConfIDim1();
   result.intrinsicDimensionality[2] = ls3D.getConfIDim2();




   result.sourceIndices[0] = ls3D.getSource2D(0);
   result.sourceIndices[1] = ls3D.getSource2D(1);

   result.orientation[0] = ls3D.phiRad();
   result.orientation[1] = ls3D.psiRad();

   result.orientationConfidence = ls3D.oriConf();

   for (int i = 0; i < 9; i++) {
      result.orientationCovariance[i] = ls3D.OrientationCovariance()[i];
   }

   result.phaseAngle = ls3D.phase();

   result.phaseConfidence = ls3D.phaseConf();

   result.phaseVariance = ls3D.PhaseVariance;

   for (int i = 0; i < 3; i++) {
      result.colorLeft[i] = ls3D.colorTripletLeft()[i];
   }
   result.colorLeftConfidence = ls3D.colConfL();

   for (int i = 0; i < 3; i++) {
      result.colorMiddle[i] = ls3D.colorTripletMiddle()[i];
   }
   result.colorMiddleConfidence = ls3D.colConfM();

   for (int i = 0; i < 3; i++) {
      result.colorRight[i] = ls3D.colorTripletRight()[i];
   }
   result.colorRightConfidence = ls3D.colConfR();

   return result;
}

/****************************************************************************************
* extendedLineSegment3D conversion functions
*
*****************************************************************************************/

extendedLineSegment3D CovisRos::toCovis(const caros_common_msgs::ECVExtendedLineSegment3D& els3D) {
   // Result
   extendedLineSegment3D result;
   
   /*
    * Copy pasta of base class converter
    */
   result.setConfidence(els3D.confidence);
   result.setLength( els3D.length);
   result.setUID(els3D.index);
   result.setPosition( KVector<double> ( els3D.position[0],
                                         els3D.position[1],
                                         els3D.position[2]));
   for (int i = 0; i < 9; i++) {
      result.PositionCovariance()[i] = els3D.positionCovariance[i];
   }
   result.setConfIDim0(els3D.intrinsicDimensionality[0]);
   result.setConfIDim1(els3D.intrinsicDimensionality[1]);
   result.setConfIDim2(els3D.intrinsicDimensionality[2]);
   result.initSource2DLineSegments( els3D.sourceIndices[0],
                                    els3D.sourceIndices[1]);
   result.setPhiRadPsiRad( els3D.orientation[0],
                           els3D.orientation[1]);
   result.setOriConf( els3D.orientationConfidence);
   for (int i = 0; i < 9; i++) {
      result.OrientationCovariance()[i] = els3D.orientationCovariance[i];
   }
   result.setPhase( els3D.phaseAngle);
   result.setPhaseConf( els3D.phaseConfidence);
   result.PhaseVariance = els3D.phaseVariance;
   for (int i = 0; i < 3; i++) {
      result.colorTripletLeft()[i] = els3D.colorLeft[i];
   }
   result.setColConfL( els3D.colorLeftConfidence);
   for (int i = 0; i < 3; i++) {
      result.colorTripletMiddle()[i] = els3D.colorMiddle[i];
   }
   result.setColConfM( els3D.colorMiddleConfidence);
   for (int i = 0; i < 3; i++) {
      result.colorTripletRight()[i] = els3D.colorRight[i];
   }
   result.setColConfR( els3D.colorRightConfidence);
   
   /*
    * The remaining fields for derived 
    */
   result.setTexlets(toCovis(els3D.patch0), toCovis(els3D.patchSide1), toCovis(els3D.patchSide2),
         static_cast<extendedLineSegmentType>(els3D.edgeType));
   result.setColor(KVector<>(els3D.color[0], els3D.color[1], els3D.color[2]));
   result.setSurfaceNormal(KVector<>(els3D.surfaceNormal[0], els3D.surfaceNormal[1], els3D.surfaceNormal[2]));
   
   return result;
}

caros_common_msgs::ECVExtendedLineSegment3D CovisRos::toRos(const extendedLineSegment3D& els3D) {
   // Result
   caros_common_msgs::ECVExtendedLineSegment3D result;
   
   /*
    * Copy pasta of base class converter
    */
   result.confidence = els3D.getConfidence();
   result.length = els3D.getLength();
   result.index = els3D.getUID();
   result.position[0] = els3D.X();
   result.position[1] = els3D.Y();
   result.position[2] = els3D.Z();
   for (int i = 0; i < 9; i++) {
      result.positionCovariance[i] = els3D.PositionCovariance()[i];
   }
   result.intrinsicDimensionality[0] = els3D.getConfIDim0();
   result.intrinsicDimensionality[1] = els3D.getConfIDim1();
   result.intrinsicDimensionality[2] = els3D.getConfIDim2();
   result.sourceIndices[0] = els3D.getSource2D(0);
   result.sourceIndices[1] = els3D.getSource2D(1);
   result.orientation[0] = els3D.phiRad();
   result.orientation[1] = els3D.psiRad();
   result.orientationConfidence = els3D.oriConf();
   for (int i = 0; i < 9; i++) {
      result.orientationCovariance[i] = els3D.OrientationCovariance()[i];
   }
   result.phaseAngle = els3D.phase();
   result.phaseConfidence = els3D.phaseConf();
   result.phaseVariance = els3D.PhaseVariance;
   for (int i = 0; i < 3; i++) {
      result.colorLeft[i] = els3D.colorTripletLeft()[i];
   }
   result.colorLeftConfidence = els3D.colConfL();
   for (int i = 0; i < 3; i++) {
      result.colorMiddle[i] = els3D.colorTripletMiddle()[i];
   }
   result.colorMiddleConfidence = els3D.colConfM();
   for (int i = 0; i < 3; i++) {
      result.colorRight[i] = els3D.colorTripletRight()[i];
   }
   result.colorRightConfidence = els3D.colConfR();   
   
   /*
    * The remaining fields for derived 
    */
   result.patch0 = toRos(els3D.getTexlet(0));
   result.patchSide1 = toRos(els3D.getTexlet(1));
   result.patchSide2 = toRos(els3D.getTexlet(2));
   result.edgeType = static_cast<uint32_t>(els3D.getEdgeType());
   const KVector<>& color = els3D.getColor();
   result.color[0] = color[0];
   result.color[1] = color[1];
   result.color[2] = color[2];
   const KVector<>& surfaceNormal = els3D.getSurfaceNormal();
   result.surfaceNormal[0] = surfaceNormal[0];
   result.surfaceNormal[1] = surfaceNormal[1];
   result.surfaceNormal[2] = surfaceNormal[2];
   
   return result;
}

/****************************************************************************************
* Texlet3D conversion functions
*
*****************************************************************************************/

texlet3D CovisRos::toCovis(const caros_common_msgs::ECVTexlet3D& tex3D) {
   texlet3D result;

   result.setConfidence(tex3D.confidence);

   result.setLength( tex3D.length);

   result.setUID(tex3D.index);

   result.setPosition( KVector<double> ( tex3D.position[0],
                                         tex3D.position[1],
                                         tex3D.position[2]));

   for (int i = 0; i < 9; i++) {
      result.PositionCovariance()[i] = tex3D.positionCovariance[i];
   }

   result.setConfIDim0(tex3D.intrinsicDimensionality[0]);
   result.setConfIDim1(tex3D.intrinsicDimensionality[1]);
   result.setConfIDim2(tex3D.intrinsicDimensionality[2]);


   result.setSourceTexlet2D( tex3D.sourceSegment);



   result.setNormalVectorPlanePatch( KVector<double> ( tex3D.orientation[0],
                                     tex3D.orientation[1],
                                     tex3D.orientation[2]));


   for (int i = 0; i < 9; i++) {
      result.OrientationCovariance()[i] = tex3D.orientationCovariance[i];
   }


   result.setColor( KVector<double> ( tex3D.color[0],
                                      tex3D.color[1],
                                      tex3D.color[2]));

   return result;
}

caros_common_msgs::ECVTexlet3D CovisRos::toRos(const texlet3D& tex3D) {
   caros_common_msgs::ECVTexlet3D result;

   result.confidence = tex3D.getConfidence();

   result.length = tex3D.getLength();

   result.index = tex3D.getUID();

   result.position[0] = tex3D.X();
   result.position[1] = tex3D.Y();
   result.position[2] = tex3D.Z();

   for (int i = 0; i < 9; i++) {
      result.positionCovariance[i] = tex3D.PositionCovariance()[i];
   }

   result.intrinsicDimensionality[0] = tex3D.getConfIDim0();
   result.intrinsicDimensionality[1] = tex3D.getConfIDim1();
   result.intrinsicDimensionality[2] = tex3D.getConfIDim2();


   result.sourceSegment = tex3D.getSourceTexlet2D();

   result.orientation[0] = tex3D.getNormalVectorPlanePatch()[0];
   result.orientation[1] = tex3D.getNormalVectorPlanePatch()[1];
   result.orientation[2] = tex3D.getNormalVectorPlanePatch()[2];

   for (int i = 0; i < 9; i++) {
      result.orientationCovariance[i] = tex3D.OrientationCovariance()[i];
   }

   for (int i = 0; i < 3; i++) {
      result.color[i] = tex3D.getColor()[i];
   }

   return result;
}



/****************************************************************************************
* Links conversion functions
*
*****************************************************************************************/



KLinkArray CovisRos::toCovis(const caros_common_msgs::ECVLinks& lnks) {
   KLinkArray result(lnks.size1, lnks.size2);

   for ( unsigned int index1 = 0; index1 < lnks.lnks.size() ; index1++ ) {
      for ( unsigned int index2 = 0; index2 < lnks.lnks[index1].trgId.size() ; index2++ ) {
         result.setCell( KPrimitiveLink( lnks.lnks.at(index1).srcId,
                                         lnks.lnks.at(index1).trgId[index2],
                                         1.0 ));
      }
   }

   return result;
}

caros_common_msgs::ECVLinks CovisRos::toRos(const KLinkArray& lnks) {
   caros_common_msgs::ECVLinks result;
   result.size1 = lnks.size1();
   result.size2 = lnks.size2();

   for ( unsigned int index1 = 0; index1 < result.size1 ; index1++ ) {
      caros_common_msgs::ECVLink lnk;
      lnk.srcId = index1;
      for ( unsigned int index2 = 0; index2 < result.size2 ; index2++ ) {
         if ( lnks.cellIndex(index1, index2) != -1 ) {
            lnk.trgId.push_back(index2);
         }
      }
      result.lnks.push_back(lnk);
   }
   return result;
}

Calibration::CameraCalibration CovisRos::toCovis(const sensor_msgs::CameraInfo& cam, bool undistorted) {

   assert(cam.binning_x == 0.0);
   assert(cam.binning_y == 0.0);
   assert(cam.roi.x_offset == 0);
   assert(cam.roi.y_offset == 0);
   assert(cam.roi.height == 0);
   assert(cam.roi.width == 0);
   assert(cam.roi.do_rectify == false);

   image_geometry::PinholeCameraModel camModel;
   camModel.fromCameraInfo(cam);

   cv::Mat P(camModel.fullProjectionMatrix());
   cv::Mat K( 3, 3, CV_64F);
   cv::Mat R( 3, 3, CV_64F);
   cv::Mat T( 3, 1, CV_64F);

   cv::decomposeProjectionMatrix( P, K, R, T);

   double angle, x, y, z;
   KMatrix<double> r;
   r.init( 3, 3, R.ptr<double>());
   KVector<> t;
   t.init( 3, T.ptr<double>());
   rotationMatrixToAxisAngle( r, angle, x, y, z);
   if ((t.norm() > 10e-6) || (angle > 10e-6)) {
      COVIS_THROW("This function can not handle camera calibration matrixes where the camera is not in the origin! Translation length: " << t.norm() << " Angle: " << angle);
   }

   Calibration::CameraCalibration result;

   result.setImageSize( camModel.fullResolution().width,
                        camModel.fullResolution().height);


   KMatrix< double > k;
   k.init( 3, 3, const_cast<double *>(cv::Mat(camModel.fullIntrinsicMatrix()).ptr<double>()) );
   result.setIntrinsicMatrix( k );
   result.setTranslationVector( KVector< double >( 0.0, 0.0, 0.0 ) );
   r.eye(1.0);
   result.setRotationMatrix( r );

   if (undistorted) {
      const double * D = camModel.distortionCoeffs().ptr<double>();
      KVector< double > d( D[0], D[1], D[2], D[3]);
      result.setDistortionVector( d);
   } else {
      result.setDistortionVector( KVector< double >( 0.0, 0.0, 0.0, 0.0 ) );
   }

   result.generateProjectionMatrix();

   return result;
}

sensor_msgs::CameraInfo CovisRos::toRos(const Calibration::CameraCalibration& cam) {
   sensor_msgs::CameraInfo result;

   KVector< double > translation = cam.getTranslation();
   double angle, x, y, z;
   rotationMatrixToAxisAngle(cam.getRotation(), angle, x, y, z);
   if ((translation.norm() > 10e-6) || (angle > 10e-6)) {
      COVIS_THROW("This function can not handle camera calibration matrixes with extrinsic parameters! Translation length: " << translation.norm() << " Angle: " << angle);
   }

   unsigned int width, height;
   cam.getImageSize( width, height);
   result.height = height;
   result.width = width;

   result.distortion_model = "plumb_bob";

   KVector< double > distortion;
   distortion = cam.getDistortion();
   result.D.resize(4);
   for (int i = 0; i < 4; i++) {
      result.D[i] = distortion[i];
   }

   for (int i = 0; i < 9; i++) {
      result.K[i] = cam.getIntrinsic()[i];
   }

   for (int i = 0; i < 12; i++) {
      result.P[i] = cam.getProjectionMatrix()[i];
   }

   for (int i = 0; i < 9; i++) {
      result.R[i] = 0.0;
   }
   result.R[0] = result.R[4] = result.R[8] = 1.0;

   // No binning
   result.binning_x = result.binning_y = 0.0;

   // No region of interest
   result.roi.x_offset   = 0;
   result.roi.y_offset   = 0;
   result.roi.height     = 0;
   result.roi.width      = 0;
   result.roi.do_rectify = false;


   return result;
}

std::pair< sensor_msgs::CameraInfo, sensor_msgs::CameraInfo > CovisRos::toRos(const Calibration::StereoCalibration& cams, double rectificationScaling) {

   KVector< double > translation = cams.leftCamera().getTranslation();
   double angle, x, y, z;
   rotationMatrixToAxisAngle(cams.leftCamera().getRotation(), angle, x, y, z);
   if ((translation.norm() > 10e-6) || (angle > 10e-6)) {
      COVIS_THROW("This function can not handle stereo camera calibration matrixes where the left camera is not in the origin! Translation length: " << translation.norm() << " Angle: " << angle);
   }

   std::cout << cams.leftCamera().getProjectionMatrix() << std::endl;
   std::cout << cams.rightCamera().getProjectionMatrix() << std::endl;

   unsigned int width, height;
   cams.leftCamera().getImageSize( width, height);
   CvSize _inputSize = cvSize( width, height);

   KMatrix<double> iL = cams.leftCamera().getIntrinsic();
   KMatrix<double> iR = cams.rightCamera().getIntrinsic();
   KVector<double> dL = cams.leftCamera().getDistortion();
   KVector<double> dR = cams.rightCamera().getDistortion();

   cv::Mat intrinsicLeft( 3, 3, CV_64F, *iL.getArray());
   cv::Mat intrinsicRight( 3, 3, CV_64F, *iR.getArray());
   cv::Mat distortionLeft( 4, 1, CV_64F, dL.getArray());
   cv::Mat distortionRight( 4, 1, CV_64F, dR.getArray());

   KMatrix<double> R_mat = cams.rightCamera().getRotation();
   KMatrix<double> T_mat = cams.rightCamera().getTranslation();
   cv::Mat R( 3, 3, CV_64F, *R_mat.getArray());
   cv::Mat T( 3, 1, CV_64F, T_mat.getArray());

   cv::Mat P1( 3, 4, CV_64F);
   cv::Mat P2( 3, 4, CV_64F);
   cv::Mat R1( 3, 3, CV_64F);
   cv::Mat R2( 3, 3, CV_64F);

   cv::Mat Q( 4, 4, CV_64F);

   cv::stereoRectify(  intrinsicLeft, distortionLeft,
                       intrinsicRight, distortionRight,
                       _inputSize,
                       R, T,
                       R1, R2,
                       P1, P2,
                       Q,
                       rectificationScaling);

   sensor_msgs::CameraInfo result_left;
   sensor_msgs::CameraInfo result_right;

   result_left.height = result_right.height = height;
   result_left.width  = result_right.width  = width;

   result_left.distortion_model = result_right.distortion_model =  "plumb_bob";

   KVector< double > distortion_left, distortion_right;
   result_left.D.resize(4);
   result_right.D.resize(4);
   for (int i = 0; i < 4; i++) {
      result_left.D[i] = dL[i];
      result_right.D[i] = dR[i];
   }

   for (int i = 0; i < 12; i++) {
      result_left.P[i] = P1.ptr<double>()[i];
      result_right.P[i] = P2.ptr<double>()[i];
   }

   for (int i = 0; i < 9; i++) {
      result_left.K[i]  = iL[i];
      result_right.K[i] = iR[i];
   }

   for (int i = 0; i < 9; i++) {
      result_left.R[i]  = R1.ptr<double>()[i];
      result_right.R[i] = R2.ptr<double>()[i];
   }

   // No binning
   result_left.binning_x      = result_left.binning_y       = 0.0;
   result_right.binning_x     = result_right.binning_y      = 0.0;

   // No region of interest
   result_left.roi.x_offset   = result_right.roi.x_offset   = 0;
   result_left.roi.y_offset   = result_right.roi.y_offset   = 0;
   result_left.roi.height     = result_right.roi.height     = 0;
   result_left.roi.width      = result_right.roi.width      = 0;
   result_left.roi.do_rectify = result_right.roi.do_rectify = false;

   return std::pair< sensor_msgs::CameraInfo, sensor_msgs::CameraInfo > ( result_left, result_right);
}


Calibration::StereoCalibration CovisRos::toCovis( sensor_msgs::CameraInfo leftCam, sensor_msgs::CameraInfo rightCam, bool rectified, bool undistorted) {

   if (rectified && !undistorted) {
      std::cout << "Rectification but no undistortion! Hmmm!" << std::endl;
      abort();
   }
   assert(leftCam.binning_x == 0.0);
   assert(leftCam.binning_y == 0.0);
   assert(leftCam.roi.x_offset == 0);
   assert(leftCam.roi.y_offset == 0);
   assert(leftCam.roi.height == 0);
   assert(leftCam.roi.width == 0);
   assert(leftCam.roi.do_rectify == false);

   assert(rightCam.binning_x == 0.0);
   assert(rightCam.binning_y == 0.0);
   assert(rightCam.roi.x_offset == 0);
   assert(rightCam.roi.y_offset == 0);
   assert(rightCam.roi.height == 0);
   assert(rightCam.roi.width == 0);
   assert(rightCam.roi.do_rectify == false);

   image_geometry::StereoCameraModel rosCams;

   rosCams.fromCameraInfo( leftCam, rightCam);

//    std::cout << rosCams.right().intrinsicMatrix() << std::endl;
//    std::cout << rosCams.right().projectionMatrix() << std::endl;
//
//    std::cout << rosCams.left().intrinsicMatrix() << std::endl;
//    std::cout << rosCams.left().projectionMatrix() << std::endl;

   Calibration::CameraCalibration resultLeft;
   Calibration::CameraCalibration resultRight;

   resultLeft.setImageSize( rosCams.left().fullResolution().width,
                            rosCams.left().fullResolution().height);

   resultRight.setImageSize( rosCams.right().fullResolution().width,
                             rosCams.right().fullResolution().height);

   if (undistorted) {
      const double * DL = rosCams.left().distortionCoeffs().ptr<double>();
      KVector< double > dl( DL[0], DL[1], DL[2], DL[3]);
      resultLeft.setDistortionVector( dl);

      const double * DR = rosCams.right().distortionCoeffs().ptr<double>();
      KVector< double > dr( DR[0], DR[1], DR[2], DR[3]);
      resultRight.setDistortionVector( dr);
   } else {
      resultLeft.setDistortionVector(KVector< double >( 0.0, 0.0, 0.0, 0.0 ) );
      resultRight.setDistortionVector(KVector< double >( 0.0, 0.0, 0.0, 0.0 ) );
   }

   if (rectified) {
      KMatrix<double> k;
      KMatrix<double> r;

      k.init( 3, 3, const_cast<double *>(cv::Mat(rosCams.left().fullIntrinsicMatrix()).ptr<double>()) );
      resultLeft.setIntrinsicMatrix( k );
      resultLeft.setTranslationVector( KVector< double >( 0.0, 0.0, 0.0 ) );
      r.eye(1.0);
      resultLeft.setRotationMatrix( r );
      resultLeft.generateProjectionMatrix();

      k.init( 3, 3, const_cast<double *>(cv::Mat(rosCams.right().fullIntrinsicMatrix()).ptr<double>()) );
      resultRight.setIntrinsicMatrix( k );
      resultRight.setTranslationVector( KVector< double >( -rosCams.baseline(), 0.0, 0.0 ) );
      r.eye(1.0);
      resultRight.setRotationMatrix( r );
      resultRight.generateProjectionMatrix();
   } else {

      COVIS_THROW("Unclear to me how to implement this. DK");

      KMatrix<double> R1;
      R1.init( 3, 3, const_cast<double*>(cv::Mat(rosCams.left().rotationMatrix()).ptr<double>()));

      KMatrix<double> R2;
      R2.init( 3, 3, const_cast<double*>(cv::Mat(rosCams.right().rotationMatrix()).ptr<double>()));

      KMatrix<double> P1;
      P1.init( 3, 4, const_cast<double*>(cv::Mat(rosCams.left().fullProjectionMatrix()).ptr<double>()));

      KMatrix<double> P2;
      P2.init( 3, 4, const_cast<double*>(cv::Mat(rosCams.right().fullProjectionMatrix()).ptr<double>()));

      std::cout << P1 << std::endl;
      std::cout << P2 << std::endl;

      std::cout << R1 * P1 << std::endl;
      std::cout << R2 * P2 << std::endl;

      std::cout << R1.inv() * P1 << std::endl;
      std::cout << R2.inv() * P2 << std::endl;

      // TODO finish this
   }

   Calibration::StereoCalibration result( resultLeft, resultRight);
   return result;
}

caros_common_msgs::CameraCalibration CovisRos::toOwnRos(const Calibration::CameraCalibration& cam) {
   caros_common_msgs::CameraCalibration result;

   result.height = cam.getHeight();
   result.width = cam.getWidth();

   for (int i = 0; i < 4; i++) {
      result.D[i] = cam.getDistortion()[i];
   }

   for (int i = 0; i < 9; i++) {
      result.K[i] = cam.getIntrinsic()[i];
   }

   for (int i = 0; i < 9; i++) {
      result.R[i] = cam.getRotation()[i];
   }

   for (int i = 0; i < 3; i++) {
      result.t[i] = cam.getTranslation()[i];
   }

   return result;
}

Calibration::CameraCalibration CovisRos::toCovis(const caros_common_msgs::CameraCalibration& cam) {
   Calibration::CameraCalibration result;

   result.setImageSize( cam.width, cam.height);

   result.setDistortionVector( KVector< double > ( cam.D[0],
                               cam.D[1],
                               cam.D[2],
                               cam.D[3]));

   KMatrix<double> K;
   K.init(3, 3, const_cast<double *> (cam.K.data()), true);
   KMatrix<double> R;
   R.init( 3, 3, const_cast<double *> (cam.R.data()), true);
   result.setIntrinsicMatrix( K);
   result.setRotationMatrix( R);

   result.setTranslationVector( KVector< double > ( cam.t[0],
                                cam.t[1],
                                cam.t[2]));
   result.generateProjectionMatrix();

   return result;
}


caros_common_msgs::StereoCalibration CovisRos::toOwnRos(const Calibration::StereoCalibration& cams) {
   caros_common_msgs::StereoCalibration result;

   result.left  = toOwnRos(cams.leftCamera());
   result.right = toOwnRos(cams.rightCamera());

   return result;
}

Calibration::StereoCalibration CovisRos::toCovis(const caros_common_msgs::StereoCalibration& cams) {
   return Calibration::StereoCalibration( toCovis(cams.left),
                                          toCovis(cams.right));
}

std::vector <lineSegment3D> CovisRos::toCovis(const caros_common_msgs::ECVLineSegments3D& lss3D) {
   std::vector <lineSegment3D> result;
   for ( unsigned int index = 0; index < lss3D.size ; index++ ) {
      result.push_back(CovisRos::toCovis(lss3D.lineSegments3D.at(index)));
   }
   return result;
}

caros_common_msgs::ECVLineSegments3D CovisRos::toRos(const  std::vector <lineSegment3D>& lss3D) {
   caros_common_msgs::ECVLineSegments3D result;
   result.size = lss3D.size();
   for ( unsigned int index = 0; index < result.size ; index++ ) {
      result.lineSegments3D.push_back(CovisRos::toRos(lss3D.at(index)));
   }
   return result;
}

std::vector <extendedLineSegment3D> CovisRos::toCovis(const caros_common_msgs::ECVExtendedLineSegments3D& elss3D) {
   std::vector <extendedLineSegment3D> result;
   for ( unsigned int index = 0; index < elss3D.size ; index++ ) {
      result.push_back(CovisRos::toCovis(elss3D.extendedLineSegments3D.at(index)));
   }
   return result;
}

caros_common_msgs::ECVExtendedLineSegments3D CovisRos::toRos(const std::vector<extendedLineSegment3D> & elss3D) {
   caros_common_msgs::ECVExtendedLineSegments3D result;
   result.size = elss3D.size();
   for ( unsigned int index = 0; index < result.size ; index++ ) {
      result.extendedLineSegments3D.push_back(CovisRos::toRos(elss3D.at(index)));
   }
   return result;
}

std::vector <texlet3D> CovisRos::toCovis(const caros_common_msgs::ECVTexlets3D& ts3D) {
   std::vector <texlet3D> result;
   for ( unsigned int index = 0; index < ts3D.size ; index++ ) {
      result.push_back(CovisRos::toCovis(ts3D.texlets3D.at(index)));
   }
   return result;
}

caros_common_msgs::ECVTexlets3D CovisRos::toRos(const  std::vector <texlet3D>& ts3D) {
   caros_common_msgs::ECVTexlets3D result;
   result.size = ts3D.size();

   for ( unsigned int index = 0; index < result.size ; index++ ) {
      result.texlets3D.push_back(CovisRos::toRos(ts3D.at(index)));
   }
   return result;
}

std::vector <texlet2D> CovisRos::toCovis(const caros_common_msgs::ECVTexlets2D& ts2D) {
   std::vector <texlet2D> result;
   for ( unsigned int index = 0; index < ts2D.size ; index++ ) {
      result.push_back(CovisRos::toCovis(ts2D.texlets2D.at(index)));
   }
   return result;
}

caros_common_msgs::ECVTexlets2D CovisRos::toRos(const  std::vector <texlet2D>& ts2D) {
   caros_common_msgs::ECVTexlets2D result;
   result.size = ts2D.size();

   for ( unsigned int index = 0; index < result.size ; index++ ) {
      result.texlets2D.push_back(CovisRos::toRos(ts2D.at(index)));
   }
   return result;
}


HighLevelFeatures::contour3D CovisRos::toCovis(const caros_common_msgs::ECVContour3D& c3D, std::vector <lineSegment3D> & lineSegments3D, const KMatrix<double>  *projectionMatrixP) {
   HighLevelFeatures::contour3D result;


   result.setPrimitveIndices(c3D.primitiveIndices);

   result.setContourUncertainty(c3D.uncertainty);
   result.setPosition( KVector<double> ( c3D.position[0],
                                         c3D.position[1],
                                         c3D.position[2]));
//    result.setOrientation(); //TODO ??? WAIL
   result.setMeanColorLeft(c3D.colorLeft[0], c3D.colorLeft[1], c3D.colorLeft[2]);
   result.setMeanColorMiddle(c3D.colorMiddle[0], c3D.colorMiddle[1], c3D.colorMiddle[2]);
   result.setMeanColorRight(c3D.colorRight[0], c3D.colorRight[1], c3D.colorRight[2]);
   std::vector<std::vector<double> > pc(3, std::vector<double>(4));
   for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 4; j++) {
         pc[i][j] = c3D.principleComponents[i*4+j];
      }
   }
   result.setPrincipalComponents(pc);
   result.setLineParameters(c3D.lineParameters);
   result.setPrimitives(lineSegments3D, c3D.primitiveIndices);
   if (projectionMatrixP != NULL) result.setProjectionMatrix(projectionMatrixP);

   return result;
}


caros_common_msgs::ECVContour3D CovisRos::toRos( HighLevelFeatures::contour3D& c3D) {
   caros_common_msgs::ECVContour3D result;

   result.primitiveIndices = c3D.getPrimitveIndices();
   result.uncertainty = c3D.getContourUncertainty();
   std::vector <double> colorLeft(3), colorRight(3), colorMiddle(3);
   c3D.calculateMeanColor();
   c3D.getMeanColorLeft(colorLeft[0], colorLeft[1], colorLeft[2]);
   c3D.getMeanColorMiddle(colorMiddle[0], colorMiddle[1], colorMiddle[2]);
   c3D.getMeanColorRight(colorRight[0], colorRight[1], colorRight[2]);
   for (int i = 0; i < 3; i++) {
      result.position[i] = c3D.getPosition()[i];
      result.colorLeft[i] = colorLeft[i];
      result.colorMiddle[i] = colorMiddle[i];
      result.colorRight[i] = colorRight[i];
   }
   for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 4; j++) {
         result.principleComponents[i*4+j] = c3D.getPrincipalComponent()[i][j];
      }
   }
   result.lineParameters = c3D.getLineParameters();


   return result;
}


HighLevelFeatures::surfling3D CovisRos::toCovis(const caros_common_msgs::ECVSurfling3D& sl3D, std::vector <texlet3D> & texlets3D) {

   HighLevelFeatures::surfling3D result;
   result.setPrimitveIndices(sl3D.primitiveIndices);
   result.setPosition( KVector<double> ( sl3D.position[0],
                                         sl3D.position[1],
                                         sl3D.position[2]));
//    result.setOrientation(); //TODO ??? WAIL
   result.setMeanColor(sl3D.color[0], sl3D.color[1], sl3D.color[2]);
   result.setBoundary(sl3D.boundry);
   result.setIsDoubleBoundaryAtNarrowPassage(sl3D.doubleBoundaryAtNarrowPassage);
   result.setBoundaryDirectionAngle(sl3D.boundaryDirectionAngle);
   result.setBoundaryAngle(sl3D.boundaryAngle);
   double a, b, c;
   result.getMeanColor(a, b, c);
   std::vector<std::vector<double> > pc(3, std::vector<double>(4));
   for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 4; j++) {
         pc[i][j] = sl3D.principleComponents[i*4+j];
      }
   }
   result.setPrincipalComponents(pc);
   result.setPrimitives( texlets3D, sl3D.primitiveIndices);

   return result;

}

HighLevelFeatures::surfling3D CovisRos::toCovis(const caros_common_msgs::ECVSurfling3D& sl3D) {

   HighLevelFeatures::surfling3D result;
   result.setPrimitveIndices(sl3D.primitiveIndices);
   result.setPosition( KVector<double> ( sl3D.position[0],
                                         sl3D.position[1],
                                         sl3D.position[2]));
//    result.setOrientation(); //TODO ??? WAIL
   result.setMeanColor(sl3D.color[0], sl3D.color[1], sl3D.color[2]);
   result.setBoundary(sl3D.boundry);
   result.setIsDoubleBoundaryAtNarrowPassage(sl3D.doubleBoundaryAtNarrowPassage);
   result.setBoundaryDirectionAngle(sl3D.boundaryDirectionAngle);
   result.setBoundaryAngle(sl3D.boundaryAngle);
   double a, b, c;
   result.getMeanColor(a, b, c);
   std::vector<std::vector<double> > pc(3, std::vector<double>(4));
   for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 4; j++) {
         pc[i][j] = sl3D.principleComponents[i*4+j];
      }
   }
   result.setPrincipalComponents(pc);
   //result.setPrimitives( texlets3D, sl3D.primitiveIndices);

   return result;

}

caros_common_msgs::ECVSurfling3D CovisRos::toRos( HighLevelFeatures::surfling3D& sl3D) {

   caros_common_msgs::ECVSurfling3D result;
   result.primitiveIndices = sl3D.getPrimitveIndices();
   result.boundry = sl3D.getIsBoundary();
   result.doubleBoundaryAtNarrowPassage = sl3D.getIsDoubleBoundaryAtNarrowPassage();
   result.boundaryDirectionAngle = sl3D.getBoundaryDirectionAngle();
   result.boundaryAngle = sl3D.getBoundaryAngle();
//    std::vector<HighLevelFeatures::boundaryType> typeVector=  sl3D.getBoundaryTypeVector(); TODO check with mila , wail
//    result.boundaryTypeVector=sl3D.getBoundaryTypeVector();
   std::vector <double> color(3, 0);
   sl3D.calculateMeanColor();
   sl3D.getMeanColor(color[0], color[1], color[2]);
   for (int i = 0; i < 3; i++) {
      result.position[i] = sl3D.getPosition()[i];
      result.color[i] = color[i];
   }
   for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 4; j++) {
         result.principleComponents[i*4+j] = sl3D.getPrincipalComponent()[i][j];
      }
   }

   return result;

}


HighLevelFeatures::surface3D CovisRos::toCovis(const caros_common_msgs::ECVSurface3D& s3D, std::vector <HighLevelFeatures::abstractSurfling*> &surflings3D) {

   HighLevelFeatures::surface3D result;
   result.setSurflingIndices(s3D.surflingIndices);
   result.setPosition( KVector<double> ( s3D.position[0],
                                         s3D.position[1],
                                         s3D.position[2]));
//    result.setOrientation(); //TODO ??? WAIL

   std::vector<std::vector<double> > pc(3, std::vector<double>(4));
   for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 4; j++) {
         pc[i][j] = s3D.principleComponents[i*4+j];
      }
   }
   result.setPrincipalComponents(pc);
   result.setSurflings(surflings3D, s3D.surflingIndices);

   return result;

}

caros_common_msgs::ECVSurface3D CovisRos::toRos( HighLevelFeatures::surface3D& s3D) {

   caros_common_msgs::ECVSurface3D result;
   result.surflingIndices = s3D.getSurflingIndices();
   for (int i = 0; i < 3; i++) {
      result.position[i] = s3D.getPosition()[i];
   }
   for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 4; j++) {
         result.principleComponents[i*4+j] = s3D.getPrincipalComponent()[i][j];
      }
   }

   return result;

}


std::vector <HighLevelFeatures::contour3D*> CovisRos::toCovis(const caros_common_msgs::ECVContours3D& cs3D, std::vector <lineSegment3D> & lineSegments3D, const KMatrix<double>  *projectionMatrixP, bool orderContours) {
   std::vector <HighLevelFeatures::contour3D*> result;
   for ( unsigned int index = 0; index < cs3D.size ; index++ ) {
      result.push_back(new HighLevelFeatures::contour3D(CovisRos::toCovis(cs3D.contours3D.at(index), lineSegments3D, projectionMatrixP)));
      result.back()->orderElements();
   }
   return result;
}

caros_common_msgs::ECVContours3D CovisRos::toRos(std::vector <HighLevelFeatures::abstractContour*>& cs3D) {
   caros_common_msgs::ECVContours3D result;
   result.size = cs3D.size();
   for ( unsigned int index = 0; index < result.size ; index++ ) {
      result.contours3D.push_back(CovisRos::toRos(*dynamic_cast< HighLevelFeatures::contour3D *>(cs3D.at(index))));
   }
   return result;
}

caros_common_msgs::ECVContours3D CovisRos::toRos(std::vector <HighLevelFeatures::contour3D*>& cs3D) {
   caros_common_msgs::ECVContours3D result;
   result.size = cs3D.size();
   for ( unsigned int index = 0; index < result.size ; index++ ) {
      result.contours3D.push_back(CovisRos::toRos(*cs3D.at(index)));
   }
   return result;
}


std::vector <HighLevelFeatures::abstractSurfling*> CovisRos::toCovis(const caros_common_msgs::ECVSurflings3D& sls3D, std::vector <texlet3D> & texlets3D) {
   std::vector <HighLevelFeatures::abstractSurfling*> result;
   for ( unsigned int index = 0; index < sls3D.size ; index++ ) {
      result.push_back(new HighLevelFeatures::surfling3D(CovisRos::toCovis(sls3D.surflings3D.at(index), texlets3D)));
   }
   return result;
}

std::vector <HighLevelFeatures::abstractSurfling*> CovisRos::toCovis(const caros_common_msgs::ECVSurflings3D& sls3D) {
   std::vector <HighLevelFeatures::abstractSurfling*> result;
   for ( unsigned int index = 0; index < sls3D.size ; index++ ) {
      result.push_back(new HighLevelFeatures::surfling3D(CovisRos::toCovis(sls3D.surflings3D.at(index))));
   }
   return result;
}

caros_common_msgs::ECVSurflings3D CovisRos::toRos( std::vector <HighLevelFeatures::abstractSurfling*>& sls3D) {
   caros_common_msgs::ECVSurflings3D result;
   result.size = sls3D.size();
   for ( unsigned int index = 0; index < result.size ; index++ ) {
      result.surflings3D.push_back(CovisRos::toRos(*dynamic_cast< HighLevelFeatures::surfling3D *>(sls3D.at(index))));
   }
   return result;
}

std::vector <HighLevelFeatures::surface3D*> CovisRos::toCovis(const caros_common_msgs::ECVSurfaces3D& ss3D, std::vector <HighLevelFeatures::abstractSurfling*> &surflings3D) {
   std::vector <HighLevelFeatures::surface3D*> result;
   for ( unsigned int index = 0; index < ss3D.size ; index++ ) {
      result.push_back(new HighLevelFeatures::surface3D(CovisRos::toCovis(ss3D.surfaces3D.at(index), surflings3D)));
      result.back()->setID(index);
   }   
   return result;
}

caros_common_msgs::ECVSurfaces3D CovisRos::toRos( std::vector <HighLevelFeatures::abstractSurface*>& ss3D) {
   caros_common_msgs::ECVSurfaces3D result;
   result.size = ss3D.size();
   for ( unsigned int index = 0; index < result.size ; index++ ) {
      result.surfaces3D.push_back(CovisRos::toRos(*dynamic_cast< HighLevelFeatures::surface3D *>(ss3D.at(index))));
   }
   return result;
}

caros_common_msgs::ECVSurfaces3D CovisRos::toRos( std::vector <HighLevelFeatures::surface3D*>& ss3D) {
   caros_common_msgs::ECVSurfaces3D result;
   result.size = ss3D.size();
   for ( unsigned int index = 0; index < result.size ; index++ ) {
      result.surfaces3D.push_back(CovisRos::toRos(*ss3D.at(index)));
   }
   return result;
}

/****************************************************************************************
* Relations conversion functions
*
*****************************************************************************************/
std::vector<std::vector<double> > CovisRos::toCovis(const caros_common_msgs::ECVRelationsMatrix& relationMatrix)
{

  std::vector < std::vector<double> > texletsNormalDistanceRelation;
  for (int i = 0; i < relationMatrix.relationsMatrix.size(); i++)
  {
    std::vector<double> RelationsVecotr;
    for (int j = 0; j < relationMatrix.relationsMatrix.at(i).relationsVector.size(); j++)
    {
      RelationsVecotr.push_back(relationMatrix.relationsMatrix.at(i).relationsVector.at(j));
    }
    texletsNormalDistanceRelation.push_back(RelationsVecotr);
  }
  return texletsNormalDistanceRelation;
}

caros_common_msgs::ECVRelationsMatrix CovisRos::toRos(const std::vector<std::vector<double> >& relationMatrix)
{

  caros_common_msgs::ECVRelationsMatrix RelationsM;
  for (uint i = 0; i < relationMatrix.size(); i++)
  {
    caros_common_msgs::ECVRelationsVector RelationsV;
    for (uint j = 0; j < relationMatrix.at(i).size(); j++)
    {
      RelationsV.relationsVector.push_back(relationMatrix.at(i).at(j));
    }
    RelationsM.relationsMatrix.push_back(RelationsV);
  }
  return RelationsM;
}


