//
//  descriptors.cpp
//  pointcloud_registration
//
//  Created by Allen Tang on 6/9/18.
//  Copyright © 2018 Allen Tang. All rights reserved.
//

#include "descriptors.hpp"

void computeNormals(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::Normal> &normals, double search_radius) {
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> norm_est;
    norm_est.setSearchMethod (tree);
    norm_est.setRadiusSearch (search_radius);
    norm_est.setInputCloud (cloud);
    norm_est.compute (normals);
}

void computeFPFHFeatures(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const pcl::PointCloud<pcl::Normal> &normals, pcl::PointCloud<pcl::FPFHSignature33> &features, double search_radius) {
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
    fpfh_est.setSearchMethod (tree);
    fpfh_est.setRadiusSearch (0.05);
    fpfh_est.setInputCloud (cloud);
    fpfh_est.setInputNormals (normals.makeShared ());
    fpfh_est.compute (features);
}
