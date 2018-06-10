//
//  descriptors.cpp
//  pointcloud_registration
//
//  Created by Allen Tang on 6/9/18.
//  Copyright © 2018 Allen Tang. All rights reserved.
//

#include "descriptors.hpp"

void computeNormals(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, pcl::PointCloud<pcl::Normal> &normals, double search_radius) {
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
    pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> norm_est;
    norm_est.setSearchMethod (tree);
    norm_est.setRadiusSearch (search_radius);
    norm_est.setInputCloud (cloud);
    norm_est.compute (normals);
}


