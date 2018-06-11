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

#define CORR_DIST 0.1
#define RANSAC_DIST 0.05
#define CONV_EPS 0.0000001

double computeICPAlignment(const pcl::PointCloud<pcl::PointXYZI>::Ptr &source, const pcl::PointCloud<pcl::PointXYZI>::Ptr &target, pcl::PointCloud<pcl::PointXYZI>::Ptr &source_aligned, Eigen::Matrix4f &T, int num_iter = 10000);

#endif /* ICP_hpp */
