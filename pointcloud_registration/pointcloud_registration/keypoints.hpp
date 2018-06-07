//
//  keypoints.hpp
//  pointcloud_registration
//
//  Created by Allen Tang on 6/7/18.
//  Copyright © 2018 Allen Tang. All rights reserved.
//

#ifndef keypoints_hpp
#define keypoints_hpp

#include <stdio.h>
#include "utilities.hpp"

double computeCloudResolution (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud);
void computeISSKeypoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &keypoints);

#endif /* keypoints_hpp */
