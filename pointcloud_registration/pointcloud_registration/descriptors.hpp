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

struct IntensitySpin
{
    float histogram[20];
};
//    EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
//} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT (IntensitySpin,
                                   (float, histogram, histogram)
                                   )

void computeNormals(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, pcl::PointCloud<pcl::Normal>::Ptr &normals, double search_radius = 0.05);


void computeFPFHFeatures(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, const pcl::PointCloud<pcl::Normal>::Ptr &normals, pcl::PointCloud<pcl::FPFHSignature33>::Ptr &features, double search_radius = 0.05);

//void computeISFeatures(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, pcl::PointCloud<IntensitySpin>::Ptr &features, double search_radius = 10.0);

#endif /* descriptors_hpp */
