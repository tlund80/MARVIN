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
 * \author Tudor Morar,Stephan Schuler, Wail Mustafa (re-factoring)
 * \file CloudMerge.h
 * \brief Utility class for merging to pcl point clouds
 */

#ifndef CLOUDMERGE_H_
#define CLOUDMERGE_H_


#include <pcl/point_types.h>
#include <pcl/registration/transforms.h>
#include <iostream>
#include <vector>
#include <iterator>

//Filter class
class CloudMerge {
   private:
//number of cameras
      int n_cameras;

      std::vector<pcl::PointCloud<pcl::PointXYZRGBA> > clouds;
//positions (using arrays because couldn't use std::vector<Eigen::Matrix4f>
//      Eigen::Matrix4f transforms[100];
      std::vector<Eigen::Matrix4f> transforms;
//transform to get uncertainty
//      Eigen::Matrix4f Unctransforms[100];
      std::vector<Eigen::Matrix4f> Unctransforms;
//position inverse
//      Eigen::Matrix4f inversetrf[100];
      std::vector<Eigen::Matrix4f> inversetrf;
//camera focal length, disparity image height, width
      float fx, fy, cx, cy, f;
      int height, width, maxheight, maxwidth;
//search radius
      float searchRadius, sqradius;

//Funtion for calculating projection of point on plane
      void calculateproj( float a, float b, float c, float d, float x0, float y0, float z0, float&x, float&y, float&z);

//function for initilising the transforms
      void init_transf();

   public:
//constructor, default values
      CloudMerge (const std::vector<pcl::PointCloud<pcl::PointXYZRGBA> > & clouds, const std::vector<Eigen::Matrix4f> & transforms);
//set search radius
      void SetRadius( float r);

//set focal length & camera center
      void SetIntrinsics (float fx0, float fy0, float cx0, float cy0);

//constructor with radius, focal length, camera center and width, height
      void SetSize (int w, int h);

      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr MergeSmoothing ();


      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr MergeMedian ();

      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr MergeAveraging ();

      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr MergeCombined ();

//Function for extimating camera centers and focal lengths
      void EstimateParameters ();
};

#endif
