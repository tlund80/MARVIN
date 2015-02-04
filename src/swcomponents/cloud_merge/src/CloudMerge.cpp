// Copyright (c) 2010, University of Southern Denmark
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the University of Southern Denmark nor the names of
//    its contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE UNIVERSITY OF SOUTHERN DENMARK BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/**
 * \author Tudor Morar, Wail Mustafa (re-factoring)
 * \file CloudMerge.cpp
 * \brief Utility class for merging to pcl point clouds
 */

#include <cloud_merge/CloudMerge.h>
//Funtion for calculating projection of point on plane
void CloudMerge::calculateproj( float a, float b, float c, float d, float x0, float y0, float z0, float&x, float&y, float&z) {
   double t = -(d + a * x0 + b * y0 + c * z0) / (a * a + b * b + c * c);

   x = x0 + a * t;
   y = y0 + b * t;
   z = z0 + c * t;
}

//function for initilising the transforms
void CloudMerge::init_transf()  {
   std::vector<Eigen::Matrix4f> inTrf = transforms;
   inversetrf.reserve(n_cameras);
   Unctransforms.reserve(n_cameras);
   for (int i = 0; i < n_cameras; i++) {
      //copy the input transform
//      transforms[i] = inTrf[i];

      //calculate inverse, to speed up search
      inversetrf[i] = inTrf[i].inverse();
      //normalize - not needed
      //inversetrf[i]=inversetrf[i]/inversetrf[i](3,3);


      //calculate uncertainty transform

      //weight for x,y direction
      float weight = 0.5;

      //delete translation
      inTrf[i](0, 3) = 0;
      inTrf[i](1, 3) = 0;
      inTrf[i](2, 3) = 0;

      //weight x,y coordinates
      inTrf[i](0, 0) = inTrf[i](0, 0) * weight;
      inTrf[i](1, 0) = inTrf[i](1, 0) * weight;
      inTrf[i](2, 0) = inTrf[i](2, 0) * weight;

      inTrf[i](0, 1) = inTrf[i](0, 1) * weight;
      inTrf[i](1, 1) = inTrf[i](1, 1) * weight;
      inTrf[i](2, 1) = inTrf[i](2, 1) * weight;

      //copy uncertainty transform
      Unctransforms[i] = inTrf[i];
   }
}

//constructor, default values
CloudMerge::CloudMerge (const std::vector<pcl::PointCloud<pcl::PointXYZRGBA> > & clouds_, const std::vector<Eigen::Matrix4f> & transforms_) {

   clouds = clouds_;
   transforms = transforms_;
   //number of cameras
   n_cameras = clouds_.size();

   //intialise parameters with default values
   fx = 525;//1386.0;//584.0;
   fy = 525;
   f = (fx + fy) / 2;

   cx = 319.5;
   cy = 239.5;

   height = 480;
   width = 640;
   maxheight = height - 5;
   maxwidth = width - 5;

   searchRadius = 0.006;
   sqradius = pow(searchRadius, 2);

   //initialise transforms
   init_transf();
}

//set search radius
void CloudMerge::SetRadius( float r) {//search radius
   searchRadius = r;
   sqradius = pow(searchRadius, 2);
}

//set focal length & camera center
void CloudMerge::SetIntrinsics (float fx0, float fy0, float cx0, float cy0) {
   //focal length
   fx = fx0;
   fy = fy0;
   f = (fx + fy) / 2;
   //image center
   cx = cx0;
   cy = cy0;
}

//constructor with radius, focal length, camera center and width, height
void CloudMerge::SetSize (int w, int h) {
   //width
   width = w;
   //height
   height = h;
   //maximum width, height
   maxheight = height - 5;
   maxwidth = width - 5;
}

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr CloudMerge::MergeSmoothing () {
   //output cloud
   pcl::PointCloud<pcl::PointXYZRGBA>::Ptr out (new pcl::PointCloud<pcl::PointXYZRGBA>);

   //-----------------------Preprocessing (apply transforms, calculate uncertainty)---------------------------------------
   //vector for inverted and multiplied uncertainties
   std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> uncertainties;
   std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> muncertainties;
   //array for transformed clouds
   std::vector<std::vector <pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> > trcloud(n_cameras);

   //#pragma omp parallel for
   for (int i = 0; i < n_cameras; i++) {
     
          std::cout << "i" << i<< std::endl;
      //cloud for uncertainties
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr uncertainty (new pcl::PointCloud<pcl::PointXYZRGBA>);
      //cloud for multiplied uncertainties
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr multunc(new pcl::PointCloud<pcl::PointXYZRGBA>(clouds[i]));
      //calculate uncertainty
      pcl::transformPointCloud(clouds[i], *uncertainty, Unctransforms[i]);

      //copy unmodified cloud
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tr(new pcl::PointCloud<pcl::PointXYZRGBA>(clouds[i]));

      //apply transform to point cloud
      pcl::transformPointCloud(clouds[i], clouds[i], transforms[i]);

      trcloud[i].resize(n_cameras);
      //invert values for uncertainty, multiply
      for (unsigned int j = 0; j < uncertainty->size(); j++) {
         uncertainty->at(j).x = 1 / (0.1 > fabs(uncertainty->at(j).x) ? 0.1 : fabs(uncertainty->at(j).x));
         uncertainty->at(j).y = 1 / (0.1 > fabs(uncertainty->at(j).y) ? 0.1 : fabs(uncertainty->at(j).y));
         uncertainty->at(j).z = 1 / (0.1 > fabs(uncertainty->at(j).z) ? 0.1 : fabs(uncertainty->at(j).z));

         multunc->at(j).x = uncertainty->at(j).x * clouds[i].at(j).x;
         multunc->at(j).y = uncertainty->at(j).y * clouds[i].at(j).y;
         multunc->at(j).z = uncertainty->at(j).z * clouds[i].at(j).z;
      }
      //add to vectors
      uncertainties.push_back(uncertainty);
      muncertainties.push_back(multunc);

      //apply transforms to get the pointcloud from all view points
      for (int j = 0; j < n_cameras; j++)
         if (i != j) {
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tr1(new pcl::PointCloud<pcl::PointXYZRGBA>(clouds[i]));
            pcl::transformPointCloud(*tr1, *tr1, inversetrf[j]);
            trcloud[i][j] = tr1;
         } else
            trcloud[i][j] = tr;
   }

   //-------------------------------Start to smooth points----------------------------------

   //Go through each point in the clouds
   for (int i = 0; i < n_cameras; i++) {
      //modified cloud
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr modcloud(new pcl::PointCloud<pcl::PointXYZRGBA>(clouds[i]));

//#pragma omp parallel for
      for (unsigned int j = 0; j < clouds[i].size(); j++) {
         //Jump over invalid points
         if (!pcl_isfinite (clouds[i].at(j).x))
            continue;

         //variables for holding found indices (much faster with preallocating memory and arrays)
         int indices[100][100];
         int count[100];
         int thecount = 0;

         //----------------------- Start of search algorithm------------------------------------------------

         //go through each depth image
        for (int k = 0; k < n_cameras; k++) {
            //calculate point coordinates in each camera coordinate system
          //  std::cout << "search" << std::endl;
            Eigen::Vector4f point = (trcloud[i][k]->at(j).getVector4fMap());
            //reset counter
            count[k] = 0;
            //int thiscount=0;
            //calculate position in depth image
            int u = point[0] / point[2] * fx + cx;
            int v = point[1] / point[2] * fy + cy;
            //calculate maximum search radius
            int maxsearch = abs(searchRadius / point[2] * f);
            //limit to speed things up, 100 candidates is enough
            maxsearch = std::min(maxsearch, 5);
            //range check
            if (u > maxwidth || u < 5 || v < 5 || v > maxheight)
               continue;

            //calculate limits, rather than distance for each point
            float mindepth = point[2] - searchRadius;
            float maxdepth = point[2] + searchRadius;
            //upper left corner of search window (in the disparity image)
            int ul = (v - maxsearch) * width + u - maxsearch;
            int maxrow = ul + 2 * maxsearch * width;
            //go through each point in the search window
            for (int thisrow = ul; thisrow <= maxrow; thisrow = thisrow + width) {
               int maxpos = thisrow + 2 * maxsearch;
	      // std::cout << "maxpos: " << maxpos << std::endl;
              for (int index = thisrow; index <= maxpos; index++){
	//	std::cout << "index: " << index << std::endl;
	//	std::cout << "k: " << k << std::endl;
	//	std::cout << "int(clouds[i].size()): " << int(clouds[i].size()) << std::endl;
	//	std::cout << "trcloud[k][k]->size(): " << int(	trcloud[k][k]-> size()) << std::endl;
	
	//	std::cout << "i: " << i << std::endl;
                  //check range
                  if (count[k]<100 && index >= 0 && index < int(clouds[i].size()) && index < int(trcloud[k][k]->size())) {
 
                     //using a cube search
                    // std::cout << trcloud[k][k]->size() << " " << index << std::endl;
                 //    if(trcloud[k][k]->size() >= index){
                     if (maxdepth > trcloud[k][k]->at(index).z && mindepth < trcloud[k][k]->at(index).z) {
		       if(k>100) std::cout << "Help k is larger tahn 100" << std::endl;
                       if(count[k]>100) std::cout << "Help count[k] is larger tahn 100" << std::endl;
      
		       indices[k][count[k]] = index;
                        thecount++;
                        count[k]++;
                     }
		
                     
                  }
                  
	      }

            }
      }

         //-------------------------------------Start of smoothing part--------------------------
         //do calculation if neibourghs found
         if (thecount > 1) {      //variables for centroid
            Eigen::Vector4f centroid, weights;
            centroid.setZero();
            weights.setZero();

            //compute centroid sum and weigths (with uncertainty)
            for (int k = 0; k < n_cameras; k++)
               for (int p = 0; p < count[k]; p++) {
                  centroid[0] += muncertainties[k]->at(indices[k][p]).x;
                  weights[0] += uncertainties[k]->at(indices[k][p]).x;
                  centroid[1] += muncertainties[k]->at(indices[k][p]).y;
                  weights[1] += uncertainties[k]->at(indices[k][p]).y;
                  centroid[2] += muncertainties[k]->at(indices[k][p]).z;
                  weights[2] += uncertainties[k]->at(indices[k][p]).z;
               }

            //final centroid values
            centroid[0] = centroid[0] / weights[0];
            centroid[1] = centroid[1] / weights[1];
            centroid[2] = centroid[2] / weights[2];
            centroid[3] = 0;

            //coveriance matrix
            EIGEN_ALIGN16 Eigen::Matrix3f covariance_matrix;
            covariance_matrix.setZero ();

            //compute covariance values
            for (int k = 0; k < n_cameras; k++)
               for (int p = 0; p < count[k]; p++) {
                  Eigen::Vector4f pt = clouds[k].points[indices[k][p]].getVector4fMap() - centroid;

                  covariance_matrix (1, 1) += pt.y () * pt.y ();
                  covariance_matrix (1, 2) += pt.y () * pt.z ();
                  covariance_matrix (2, 2) += pt.z () * pt.z ();
                  pt *= pt.x ();
                  covariance_matrix (0, 0) += pt.x ();
                  covariance_matrix (0, 1) += pt.y ();
                  covariance_matrix (0, 2) += pt.z ();
               }
            covariance_matrix (1, 0) = covariance_matrix (0, 1);
            covariance_matrix (2, 0) = covariance_matrix (0, 2);
            covariance_matrix (2, 1) = covariance_matrix (1, 2);

            //calculate plane parameters by finding eigenvalues
            /*EIGEN_ALIGN16 Eigen::Vector3f eigen_values;
            EIGEN_ALIGN16 Eigen::Matrix3f eigen_vectors;
            pcl::eigen33 (covariance_matrix, eigen_vectors, eigen_values);*/

            //Eigen::EigenSolver<Eigen::Matrix3f> ei_symm (covariance_matrix);

            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> ei_symm (covariance_matrix);
            EIGEN_ALIGN16 Eigen::Matrix3f eigen_vectors = ei_symm.eigenvectors();

            //plane parameters vector
            Eigen::Vector4f plane_parameters;

            // The normalization is not necessary, since the eigenvectors from libeigen are already normalized
            plane_parameters[0] = eigen_vectors (0, 0);
            plane_parameters[1] = eigen_vectors (1, 0);
            plane_parameters[2] = eigen_vectors (2, 0);
            plane_parameters[3] = 0;

            // Hessian form (D = nc . p_plane (centroid here) + p)
            plane_parameters[3] = -1 * plane_parameters.dot (centroid);

            //move point to plane
            pcl::PointXYZRGBA p;
            calculateproj(plane_parameters[0], plane_parameters[1], plane_parameters[2], plane_parameters[3], clouds[i].at(j).x, clouds[i].at(j).y, clouds[i].at(j).z, p.x, p.y, p.z);
            modcloud->at(j).x = p.x;
            modcloud->at(j).y = p.y;
            modcloud->at(j).z = p.z;
         }
         
      }
       
      if (out->size() == 0)
         *out = *modcloud;
      else
         *out += *modcloud;
      }
   return(out);
}


pcl::PointCloud<pcl::PointXYZRGBA>::Ptr CloudMerge::MergeMedian () {
   //output cloud
   pcl::PointCloud<pcl::PointXYZRGBA>::Ptr out (new pcl::PointCloud<pcl::PointXYZRGBA>);

   //-----------------------Preprocessing (apply transforms)---------------------------------------
   //array for transformed clouds
   std::vector<std::vector <pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> > trcloud(n_cameras);

   //#pragma omp parallel for
   for (int i = 0; i < n_cameras; i++) {
      //copy unmodified cloud
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tr(new pcl::PointCloud<pcl::PointXYZRGBA>(clouds[i]));

      //apply transform to point cloud
      pcl::transformPointCloud(clouds[i], clouds[i], transforms[i]);

      trcloud[i].resize(n_cameras);

      //apply transforms to get the pointcloud from all view points
      for (int j = 0; j < n_cameras; j++)
         if (i != j) {
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tr1(new pcl::PointCloud<pcl::PointXYZRGBA>(clouds[i]));
            pcl::transformPointCloud(*tr1, *tr1, inversetrf[j]);
            trcloud[i][j] = tr1;
         } else
            trcloud[i][j] = tr;
   }

   //-------------------------------Start to smooth points----------------------------------

   //Go through each point in the clouds
   for (int i = 0; i < n_cameras; i++) {
      //modified cloud
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr modcloud(new pcl::PointCloud<pcl::PointXYZRGBA>(clouds[i]));

//#pragma omp parallel for
      for (unsigned int j = 0; j < clouds[i].size(); j++) {
         //Jump over invalid points
         if (!pcl_isfinite (clouds[i].at(j).x))
            continue;
         //variables for holding found indices (much faster with preallocating memory and arrays)
         //float x[300], y[300], z[300];
         //TODO: check if it is possible to use c arrays, Wail.
         std::vector<float> x(300);
         std::vector<float> y(300);
         std::vector<float> z(300);
         int thecount = 0;

         //----------------------- Start of search algorithm------------------------------------------------

         //go through each depth image
         for (int k = 0; k < n_cameras; k++) {
            //calculate point coordinates in each camera coordinate system
            Eigen::Vector4f point = (trcloud[i][k]->at(j).getVector4fMap());
            //calculate position in depth image
            int u = point[0] / point[2] * fx + cx;
            int v = point[1] / point[2] * fy + cy;
            //calculate maximum search radius
            int maxsearch = abs(searchRadius / point[2] * f);
            //limit to speed things up, 100 candidates is enough
            maxsearch = std::min(maxsearch, 5);
            //range check
            if (u > maxwidth || u < 5 || v < 5 || v > maxheight)
               continue;

            //calculate limits, rather than distance for each point
            float mindepth = point[2] - searchRadius;
            float maxdepth = point[2] + searchRadius;
            //upper left corner of search window (in the disparity image)
            int ul = (v - maxsearch) * width + u - maxsearch;
            int maxrow = ul + 2 * maxsearch * width;
            //go through each point in the search window
            for (int thisrow = ul; thisrow <= maxrow; thisrow = thisrow + width) {
               int maxpos = thisrow + 2 * maxsearch;
               for (int index = thisrow; index <= maxpos; index++)
                  //check range
                  if (index >= 0 && index < int(clouds[i].size()) && index < int(trcloud[k][k]->size())) {
                     //if distance is smaller, store point
                     if (maxdepth > trcloud[k][k]->at(index).z && mindepth < trcloud[k][k]->at(index).z) { //using a cube search
                        x[thecount] = (clouds[k].at(index).x);
                        y[thecount] = (clouds[k].at(index).y);
                        z[thecount] = (clouds[k].at(index).z);

                        thecount++;
                     }
                  }
            }
         }

         //-------------------------------------Start of smoothing part--------------------------
         //do calculation if neibourghs found
         if (thecount > 1) {
            //sort coordinate vectors
            std::sort(x.begin(), x.begin() + thecount);
            std::sort(x.begin(), x.begin() + thecount);
            std::sort(x.begin(), x.begin() + thecount);

            //calculate median
            int med = thecount / 2;

            //assign median
            modcloud->at(j).x = x[med];
            modcloud->at(j).y = y[med];
            modcloud->at(j).z = z[med];
         }
      }
      if (out->size() == 0)
         *out = *modcloud;
      else
         *out += *modcloud;
   }

   return(out);
}

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr CloudMerge::MergeAveraging () {
   //output cloud
   pcl::PointCloud<pcl::PointXYZRGBA>::Ptr out (new pcl::PointCloud<pcl::PointXYZRGBA>);

   //counter for total modified points
   //int totalcount=0;

   //-----------------------Preprocessing (apply transforms, calculate uncertainty)---------------------------------------
   //vector for inverted and multiplied uncertainties
   std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> uncertainties;
   std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> muncertainties;
   //array for transformed clouds
   std::vector<std::vector <pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> > trcloud(n_cameras);

   //#pragma omp parallel for
   for (int i = 0; i < n_cameras; i++) {
      //cloud for uncertainties
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr uncertainty (new pcl::PointCloud<pcl::PointXYZRGBA>);
      //cloud for multiplied uncertainties
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr multunc(new pcl::PointCloud<pcl::PointXYZRGBA>(clouds[i]));
      //calculate uncertainty
      pcl::transformPointCloud(clouds[i], *uncertainty, Unctransforms[i]);

      //copy unmodified cloud
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tr(new pcl::PointCloud<pcl::PointXYZRGBA>(clouds[i]));

      //apply transform to point cloud
      pcl::transformPointCloud(clouds[i], clouds[i], transforms[i]);

      trcloud[i].resize(n_cameras);
      //invert values for uncertainty, multiply
      for (unsigned int j = 0; j < uncertainty->size(); j++) {
         uncertainty->at(j).x = 1 / (0.1 > fabs(uncertainty->at(j).x) ? 0.1 : fabs(uncertainty->at(j).x));
         uncertainty->at(j).y = 1 / (0.1 > fabs(uncertainty->at(j).y) ? 0.1 : fabs(uncertainty->at(j).y));
         uncertainty->at(j).z = 1 / (0.1 > fabs(uncertainty->at(j).z) ? 0.1 : fabs(uncertainty->at(j).z));

         multunc->at(j).x = uncertainty->at(j).x * clouds[i].at(j).x;
         multunc->at(j).y = uncertainty->at(j).y * clouds[i].at(j).y;
         multunc->at(j).z = uncertainty->at(j).z * clouds[i].at(j).z;
      }
      //add to vectors
      uncertainties.push_back(uncertainty);
      muncertainties.push_back(multunc);

      //apply transforms to get the pointcloud from the necessary viewpoints
      for (int j = i; j < n_cameras; j++)
         if (i != j) {
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tr1(new pcl::PointCloud<pcl::PointXYZRGBA>(clouds[i]));
            pcl::transformPointCloud(*tr1, *tr1, inversetrf[j]);
            trcloud[i][j] = tr1;
         } else
            trcloud[i][j] = tr;
   }

   //-------------------------------Start to smooth points----------------------------------

   //Go through each point in the clouds
   for (int i = 0; i < n_cameras; i++) {
      //modified cloud
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr modcloud(new pcl::PointCloud<pcl::PointXYZRGBA>(clouds[i]));

//#pragma omp parallel for
      for (unsigned int j = 0; j < clouds[i].size(); j++) {
         //Jump over invalid points
         if (!pcl_isfinite (clouds[i].at(j).x))
            continue;

         //variables for holding found indices (much faster with preallocating memory and arrays)
         int indices[100];
         int trigger = 0;

         //----------------------- Start of search algorithm------------------------------------------------

         //go through each depth image
         for (int k = i + 1; k < n_cameras; k++) {
            //calculate point coordinates in each camera coordinate system
            Eigen::Vector4f point = (trcloud[i][k]->at(j).getVector4fMap());
            //init indices
            indices[k] = -1;
            //initialise closest dist
            float closestDist = searchRadius;
            //calculate position in depth image
            int u = point[0] / point[2] * fx + cx;
            int v = point[1] / point[2] * fy + cy;
            //calculate maximum search radius
            int maxsearch = abs(searchRadius / point[2] * f);
            //limit to speed things up, 100 candidates is enough
            maxsearch = std::min(maxsearch, 5);
            //range check
            if (u > maxwidth || u < 5 || v < 5 || v > maxheight)
               continue;

            //calculate limits, rather than distance for each point
            float mindepth = point[2] - searchRadius;
            float maxdepth = point[2] + searchRadius;
            //upper left corner of search window (in the disparity image)
            int ul = (v - maxsearch) * width + u - maxsearch;
            int maxrow = ul + 2 * maxsearch * width;
            //go through each point in the search window
            for (int thisrow = ul; thisrow <= maxrow; thisrow = thisrow + width) {
               int maxpos = thisrow + 2 * maxsearch;
               for (int index = thisrow; index <= maxpos; index++)
                  //check range
                  if (index >= 0 && index < int(clouds[i].size()) && index < int(trcloud[k][k]->size())) {
                     //depth check
                     if (maxdepth > trcloud[k][k]->at(index).z && mindepth < trcloud[k][k]->at(index).z) { //using a cube search
                        float dist = sqrt(std::pow(clouds[i].at(j).x - clouds[k].at(index).x, 2) + std::pow(clouds[i].at(j).y - clouds[k].at(index).y, 2) + std::pow(clouds[i].at(j).z - clouds[k].at(index).z, 2));
                        if (dist < closestDist) {
                           closestDist = dist;
                           indices[k] = index;
                        }
                        trigger = 1;
                     }
                  }
            }
         }

         //-------------------------------------Start of averaging part--------------------------
         //do calculation if neibourghs found
         if (trigger == 1) {      //variables for centroid
            Eigen::Vector4f centroid, weights;
            centroid.setZero();
            weights.setZero();


            //add neibourghs
            for (int k = i + 1; k < n_cameras; k++)
               if (indices[k] != -1) {
                  //add to centroid and weight
                  centroid[0] += muncertainties[k]->at(indices[k]).x;
                  weights[0] += uncertainties[k]->at(indices[k]).x;
                  centroid[1] += muncertainties[k]->at(indices[k]).y;
                  weights[1] += uncertainties[k]->at(indices[k]).y;
                  centroid[2] += muncertainties[k]->at(indices[k]).z;
                  weights[2] += uncertainties[k]->at(indices[k]).z;
                  //delete the point so it's not found again
                  clouds[k].at(indices[k]).x = clouds[k].at(indices[k]).y =
                                                  clouds[k].at(indices[k]).z = std::numeric_limits<float>::quiet_NaN ();
                  trcloud[k][k]->at(indices[k]).x = trcloud[k][k]->at(indices[k]).y =
                                                       trcloud[k][k]->at(indices[k]).z = std::numeric_limits<float>::quiet_NaN ();
               }

            //also add the point
            centroid[0] += muncertainties[i]->at(j).x;
            weights[0] += uncertainties[i]->at(j).x;
            centroid[1] += muncertainties[i]->at(j).y;
            weights[1] += uncertainties[i]->at(j).y;
            centroid[2] += muncertainties[i]->at(j).z;
            weights[2] += uncertainties[i]->at(j).z;

            //set values
            modcloud->at(j).x = centroid[0] / weights[0];
            modcloud->at(j).y = centroid[1] / weights[1];
            modcloud->at(j).z = centroid[2] / weights[2];

            //increase counter for modified points
            //totalcount++;
         }
      }
      if (out->size() == 0)
         *out = *modcloud;
      else
         *out += *modcloud;
   }

   //output total number of modified points
   //std::cout<<totalcount<<";";


   return(out);
}

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr CloudMerge::MergeCombined () {
   //output cloud
   pcl::PointCloud<pcl::PointXYZRGBA>::Ptr out (new pcl::PointCloud<pcl::PointXYZRGBA>);

   //-----------------------Preprocessing (apply transforms, calculate uncertainty)---------------------------------------
   //vector for inverted and multiplied uncertainties
   std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> uncertainties;
   std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> muncertainties;
   //array for transformed clouds
   std::vector<std::vector <pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> > trcloud(n_cameras);

   //#pragma omp parallel for
   for (int i = 0; i < n_cameras; i++) {
      //cloud for uncertainties
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr uncertainty (new pcl::PointCloud<pcl::PointXYZRGBA>);
      //cloud for multiplied uncertainties
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr multunc(new pcl::PointCloud<pcl::PointXYZRGBA>(clouds[i]));
      //calculate uncertainty
      pcl::transformPointCloud(clouds[i], *uncertainty, Unctransforms[i]);

      //copy unmodified cloud
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tr(new pcl::PointCloud<pcl::PointXYZRGBA>(clouds[i]));

      //apply transform to point cloud
      pcl::transformPointCloud(clouds[i], clouds[i], transforms[i]);

      trcloud[i].resize(n_cameras);
      //invert values for uncertainty, multiply
      for (unsigned int j = 0; j < uncertainty->size(); j++) {
         uncertainty->at(j).x = 1 / (0.1 > fabs(uncertainty->at(j).x) ? 0.1 : fabs(uncertainty->at(j).x));
         uncertainty->at(j).y = 1 / (0.1 > fabs(uncertainty->at(j).y) ? 0.1 : fabs(uncertainty->at(j).y));
         uncertainty->at(j).z = 1 / (0.1 > fabs(uncertainty->at(j).z) ? 0.1 : fabs(uncertainty->at(j).z));

         multunc->at(j).x = uncertainty->at(j).x * clouds[i].at(j).x;
         multunc->at(j).y = uncertainty->at(j).y * clouds[i].at(j).y;
         multunc->at(j).z = uncertainty->at(j).z * clouds[i].at(j).z;
      }
      //add to vectors
      uncertainties.push_back(uncertainty);
      muncertainties.push_back(multunc);

      //apply transforms to get the pointcloud from the necessary viewpoints
      for (int j = 0; j < n_cameras; j++)
         if (i != j) {
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tr1(new pcl::PointCloud<pcl::PointXYZRGBA>(clouds[i]));
            pcl::transformPointCloud(*tr1, *tr1, inversetrf[j]);
            trcloud[i][j] = tr1;
         } else
            trcloud[i][j] = tr;
   }

   //-------------------------------Start to average & smooth points----------------------------------

   //Go through each point in the clouds
   for (int i = 0; i < n_cameras; i++) {
      //modified cloud
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr modcloud(new pcl::PointCloud<pcl::PointXYZRGBA>(clouds[i]));

      //#pragma omp parallel for
      for (unsigned int j = 0; j < clouds[i].size(); j++) {
         //Jump over invalid points
         if (!pcl_isfinite (clouds[i].at(j).x))
            continue;

         //variables for holding found indices (much faster with preallocating memory and arrays)
         int indices[100];
         int trigger = 0;
         int smthindices[100][100];
         int count[100];
         int thecount = 0;


         //----------------------- Start of search algorithm------------------------------------------------

         //go through each depth image
         for (int k = 0; k < n_cameras; k++) {
            //calculate point coordinates in each camera coordinate system
            Eigen::Vector4f point = (trcloud[i][k]->at(j).getVector4fMap());
            //init indices
            indices[k] = -1;
            count[k] = 0;
            //initialise closest dist
            float closestDist = sqradius;
            //calculate position in depth image
            int u = point[0] / point[2] * fx + cx;
            int v = point[1] / point[2] * fy + cy;
            //calculate maximum search radius
            int maxsearch = abs(searchRadius / point[2] * f);
            //limit to speed things up, 100 candidates is enough
            maxsearch = std::min(maxsearch, 5);
            //range check
            if (u > maxwidth || u < 5 || v < 5 || v > maxheight)
               continue;
            //calculate limits, rather than distance for each point
            float mindepth = point[2] - searchRadius;
            float maxdepth = point[2] + searchRadius;
            //upper left corner of search window (in the disparity image)
            int ul = (v - maxsearch) * width + u - maxsearch;
            int maxrow = ul + 2 * maxsearch * width;
            //go through each point in the search window
            for (int thisrow = ul; thisrow <= maxrow; thisrow = thisrow + width) {
               int maxpos = thisrow + 2 * maxsearch;
               for (int index = thisrow; index <= maxpos; index++)
                  //check range
                  if (index >= 0 && index < int(clouds[i].size()) && index < int(trcloud[k][k]->size())) {
                     //depth check
                     if (maxdepth > trcloud[k][k]->at(index).z && mindepth < trcloud[k][k]->at(index).z) { //using a cube search
                        //averaging part
                        if (k > i) {
                           float dist = std::pow(clouds[i].at(j).x - clouds[k].at(index).x, 2) + std::pow(clouds[i].at(j).y - clouds[k].at(index).y, 2) + std::pow(clouds[i].at(j).z - clouds[k].at(index).z, 2);
                           if (dist < closestDist) {
                              closestDist = dist;
                              indices[k] = index;
                           }
                           trigger = 1;
                        }

                        //smoothing part index storage
                        smthindices[k][count[k]] = index;
                        thecount++;
                        count[k]++;
                     }
                  }
            }
         }


         //-------------------------------------Start of averaging part--------------------------
         //do calculation if neibourghs found
         if (trigger == 1) {      //variables for centroid
            Eigen::Vector4f centroid, weights;
            centroid.setZero();
            weights.setZero();


            //add neibourghs
            for (int k = i + 1; k < n_cameras; k++)
               if (indices[k] != -1) {
                  //add to centroid and weight
                  centroid[0] += muncertainties[k]->at(indices[k]).x;
                  weights[0] += uncertainties[k]->at(indices[k]).x;
                  centroid[1] += muncertainties[k]->at(indices[k]).y;
                  weights[1] += uncertainties[k]->at(indices[k]).y;
                  centroid[2] += muncertainties[k]->at(indices[k]).z;
                  weights[2] += uncertainties[k]->at(indices[k]).z;
                  //delete the point so it's not found again
                  clouds[k].at(indices[k]).x = clouds[k].at(indices[k]).y =
                                                  clouds[k].at(indices[k]).z = std::numeric_limits<float>::quiet_NaN ();
                  trcloud[k][k]->at(indices[k]).x = trcloud[k][k]->at(indices[k]).y =
                                                       trcloud[k][k]->at(indices[k]).z = std::numeric_limits<float>::quiet_NaN ();
               }

            //also add the point
            centroid[0] += muncertainties[i]->at(j).x;
            weights[0] += uncertainties[i]->at(j).x;
            centroid[1] += muncertainties[i]->at(j).y;
            weights[1] += uncertainties[i]->at(j).y;
            centroid[2] += muncertainties[i]->at(j).z;
            weights[2] += uncertainties[i]->at(j).z;

            //set values
            modcloud->at(j).x = centroid[0] / weights[0];
            modcloud->at(j).y = centroid[1] / weights[1];
            modcloud->at(j).z = centroid[2] / weights[2];
         }


         //---------------------------Smoothing calculation--------------------------------------
         if (thecount > 1) {      //variables for centroid
            Eigen::Vector4f centroid1, weights1;
            centroid1.setZero();
            weights1.setZero();
            //compute centroid sum and weigths (with uncertainty)
            for (int k = 0; k < n_cameras; k++)
               for (int p = 0; p < count[k]; p++) {
                  centroid1[0] += muncertainties[k]->at(smthindices[k][p]).x;
                  weights1[0] += uncertainties[k]->at(smthindices[k][p]).x;
                  centroid1[1] += muncertainties[k]->at(smthindices[k][p]).y;
                  weights1[1] += uncertainties[k]->at(smthindices[k][p]).y;
                  centroid1[2] += muncertainties[k]->at(smthindices[k][p]).z;
                  weights1[2] += uncertainties[k]->at(smthindices[k][p]).z;
               }

            //final centroid values
            centroid1[0] = centroid1[0] / weights1[0];
            centroid1[1] = centroid1[1] / weights1[1];
            centroid1[2] = centroid1[2] / weights1[2];
            centroid1[3] = 0;

            //coveriance matrix
            EIGEN_ALIGN16 Eigen::Matrix3f covariance_matrix;
            covariance_matrix.setZero ();

            //compute covariance values
            for (int k = 0; k < n_cameras; k++)
               for (int p = 0; p < count[k]; p++)
                  if (pcl_isfinite ( clouds[k].points[smthindices[k][p]].x)) {
                     Eigen::Vector4f pt = clouds[k].points[smthindices[k][p]].getVector4fMap() - centroid1;

                     covariance_matrix (1, 1) += pt.y () * pt.y ();
                     covariance_matrix (1, 2) += pt.y () * pt.z ();
                     covariance_matrix (2, 2) += pt.z () * pt.z ();
                     pt *= pt.x ();
                     covariance_matrix (0, 0) += pt.x ();
                     covariance_matrix (0, 1) += pt.y ();
                     covariance_matrix (0, 2) += pt.z ();
                  }
            covariance_matrix (1, 0) = covariance_matrix (0, 1);
            covariance_matrix (2, 0) = covariance_matrix (0, 2);
            covariance_matrix (2, 1) = covariance_matrix (1, 2);

            //calculate plane parameters by finding eigenvalues

            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> ei_symm (covariance_matrix);
            EIGEN_ALIGN16 Eigen::Matrix3f eigen_vectors = ei_symm.eigenvectors();

            //plane parameters vector
            Eigen::Vector4f plane_parameters;

            // The normalization is not necessary, since the eigenvectors from libeigen are already normalized
            plane_parameters[0] = eigen_vectors (0, 0);
            plane_parameters[1] = eigen_vectors (1, 0);
            plane_parameters[2] = eigen_vectors (2, 0);
            plane_parameters[3] = 0;

            // Hessian form (D = nc . p_plane (centroid here) + p)
            plane_parameters[3] = -1 * plane_parameters.dot (centroid1);
            //now move point to smoothed plane

            pcl::PointXYZRGBA p;
            calculateproj(plane_parameters[0], plane_parameters[1], plane_parameters[2], plane_parameters[3], modcloud->at(j).x, modcloud->at(j).y, modcloud->at(j).z, p.x, p.y, p.z);
            modcloud->at(j).x = p.x;
            modcloud->at(j).y = p.y;
            modcloud->at(j).z = p.z;
         }

      }
      if (out->size() == 0)
         *out = *modcloud;
      else
         *out += *modcloud;
   }

   //output total number of modified points
   //std::cout<<totalcount<<";";
   return(out);
}

//Function for extimating camera centers and focal lengths
void CloudMerge::EstimateParameters () {
   //sums, counter and point coordinate
   float sumfx = 0, sumfy = 0, sumcx = 0, sumcy = 0;
   int avgcount = 0;
   float x1, x2, y1, y2, z1, z2;
   int u1, u2, v1, v2;

   for (unsigned int i = 0; i < clouds.size(); i++) {  //intialize index
      unsigned int j = 0, k = clouds[i].size() / 2;
      while (k < clouds[i].size()) {
         //Jump over invalid points
         while (!pcl_isfinite (clouds[i].at(j).x))
            j++;

         //Jump over invalid points
         while ( (k < clouds[i].size() && !pcl_isfinite (clouds[i].at(k).x)) ||
                 j % width == k % width ||
                 (int)j / width == (int)k / width)
            k++;

         if (k < clouds[i].size()) {
            //assign values for point j
            x1 = clouds[i].at(j).x;
            y1 = clouds[i].at(j).y;
            z1 = clouds[i].at(j).z;
            u1 = j % width;
            v1 = j / width;

            //assign values for point k
            x2 = clouds[i].at(k).x;
            y2 = clouds[i].at(k).y;
            z2 = clouds[i].at(k).z;
            u2 = k % width;
            v2 = k / width;

            //apply formulas
            float cx0 = (u2 * x1 / x2 * z2 / z1 - u1) / (x1 / x2 * z2 / z1 - 1);
            float cy0 = (v2 * y1 / y2 * z2 / z1 - v1) / (y1 / y2 * z2 / z1 - 1);
            float fx0 = (u1 - cx0) * z1 / x1;
            float fy0 = (v1 - cy0) * z1 / y1;

            //add to sums
            sumcx = sumcx + cx0;
            sumcy = sumcy + cy0;
            sumfx = sumfx + fx0;
            sumfy = sumfy + fy0;

            //increase counters
            avgcount++;
            j++;
            k++;
         }
      }
   }
   //output result
   std::cout << "cx=" << sumcx / avgcount << "  cy=" << sumcy / avgcount << "  fx=" << sumfx / avgcount << "  fy=" << sumfy / avgcount << std::endl;
}


