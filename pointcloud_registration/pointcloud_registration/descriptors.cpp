//
//  descriptors.cpp
//  pointcloud_registration
//
//  Created by Allen Tang on 6/9/18.
//  Copyright © 2018 Allen Tang. All rights reserved.
//

#include "descriptors.hpp"

void computeNormals(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, pcl::PointCloud<pcl::Normal>::Ptr &normals, double search_radius) {
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
    pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> norm_est;
    norm_est.setSearchMethod (tree);
    norm_est.setKSearch(4);
//    norm_est.setRadiusSearch (search_radius);
    norm_est.setInputCloud (cloud);
    norm_est.compute (*normals);
}

void computeFPFHFeatures(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, const pcl::PointCloud<pcl::Normal>::Ptr &normals, pcl::PointCloud<pcl::FPFHSignature33>::Ptr &features, double search_radius) {
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
    pcl::FPFHEstimation<pcl::PointXYZI, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
    fpfh_est.setSearchMethod (tree);
    fpfh_est.setKSearch(4);
//    fpfh_est.setRadiusSearch (search_radius);
    fpfh_est.setInputCloud (cloud);
    fpfh_est.setInputNormals (normals->makeShared ());
    fpfh_est.compute (*features);
}

void computeISFeatures(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, pcl::PointCloud<IntensitySpin>::Ptr &features, double search_radius){
    pcl::IntensitySpinEstimation<pcl::PointXYZI, IntensitySpin> ispin_est;
    pcl::search::KdTree<pcl::PointXYZI>::Ptr treept3 (new pcl::search::KdTree<pcl::PointXYZI> (false));
    ispin_est.setSearchMethod (treept3);
    ispin_est.setRadiusSearch (search_radius);
    ispin_est.setNrDistanceBins (4);
    ispin_est.setNrIntensityBins (5);
    
    ispin_est.setInputCloud (cloud->makeShared ());
    pcl::PointCloud<IntensitySpin> ispin_output;
    ispin_est.compute (*features);
}

void computeRIFTFeatures(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, pcl::PointCloud<RIFT>::Ptr &features, double search_radius) {
    // surface normals
    pcl::PointCloud<pcl::Normal>::Ptr cloud_n (new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> norm_est;
    norm_est.setInputCloud(cloud);
    pcl::search::KdTree<pcl::PointXYZI>::Ptr treept1 (new pcl::search::KdTree<pcl::PointXYZI> (false));
    norm_est.setSearchMethod(treept1);
    norm_est.setRadiusSearch(search_radius);
    norm_est.compute(*cloud_n);
    
    // intensity gradient
    pcl::PointCloud<pcl::IntensityGradient>::Ptr cloud_ig (new pcl::PointCloud<pcl::IntensityGradient>);
    pcl::IntensityGradientEstimation<pcl::PointXYZI, pcl::Normal, pcl::IntensityGradient> gradient_est;
    gradient_est.setInputCloud(cloud);
    gradient_est.setInputNormals(cloud_n);
    pcl::search::KdTree<pcl::PointXYZI>::Ptr treept2 (new pcl::search::KdTree<pcl::PointXYZI> (false));
    gradient_est.setSearchMethod(treept2);
    gradient_est.setRadiusSearch(search_radius);
    gradient_est.compute(*cloud_ig);
    
    // rift features
    pcl::RIFTEstimation<pcl::PointXYZI, pcl::IntensityGradient, RIFT> rift_est;
    pcl::search::KdTree<pcl::PointXYZI>::Ptr treept3 (new pcl::search::KdTree<pcl::PointXYZI> (false));
    rift_est.setSearchMethod(treept3);
    rift_est.setRadiusSearch(search_radius);
    rift_est.setNrDistanceBins(4);
    rift_est.setNrGradientBins(8);
    rift_est.setInputGradient(cloud_ig);
    rift_est.setInputCloud(cloud->makeShared());
    pcl::PointCloud<RIFT> rift_output;
    rift_est.compute (*features);
    
}
