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

#define CORR_DIST 0.01
#define RANSAC_DIST 0.01
#define CONV_EPS 0.00001

void computeICPAlignment(const pcl::PointCloud<pcl::PointXYZI>::Ptr &source, const pcl::PointCloud<pcl::PointXYZI>::Ptr &target, pcl::PointCloud<pcl::PointXYZI>::Ptr &source_aligned, Eigen::Matrix4f &T, int num_iter = 10000);

#endif /* ICP_hpp */
