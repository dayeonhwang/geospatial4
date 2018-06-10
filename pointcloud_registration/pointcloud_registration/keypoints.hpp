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

double computeCloudResolution (const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &cloud);
void extractIndices(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr &result,pcl::PointIndicesConstPtr indices);
void computeISSKeypoints(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr &keypoints);
void computeSIFTKeypoints(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr &keypoints);
void computeHARRISKeypoints(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr &keypoints);


#endif /* keypoints_hpp */
