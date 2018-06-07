//
//  ICP.hpp
//  pointcloud_registration
//
//  Created by Allen Tang on 6/7/18.
//  Copyright © 2018 Allen Tang. All rights reserved.
//

#ifndef ICP_hpp
#define ICP_hpp

#include <stdio.h>
#include "utilities.hpp"

#define ITER 10000
#define CORR_DIST 0.1
#define RANSAC_DIST 0.1
#define CONV_EPS 0.0001

void computeICPAlignment(const pcl::PointCloud<pcl::PointXYZ>::Ptr &source, const pcl::PointCloud<pcl::PointXYZ>::Ptr &target, pcl::PointCloud<pcl::PointXYZ>::Ptr &source_aligned, Eigen::Matrix4f &T);

#endif /* ICP_hpp */
