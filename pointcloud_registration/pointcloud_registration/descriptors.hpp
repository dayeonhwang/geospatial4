//
//  descriptors.hpp
//  pointcloud_registration
//
//  Created by Allen Tang on 6/9/18.
//  Copyright © 2018 Allen Tang. All rights reserved.
//

#ifndef descriptors_hpp
#define descriptors_hpp

#include <stdio.h>
#include "utilities.hpp"

void computeNormals(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::Normal> &normals, double search_radius = 0.05);


void computeFPFHFeatures(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const pcl::PointCloud<pcl::Normal> &normals, pcl::PointCloud<pcl::FPFHSignature33> &features, double search_radius = 0.05);

#endif /* descriptors_hpp */
