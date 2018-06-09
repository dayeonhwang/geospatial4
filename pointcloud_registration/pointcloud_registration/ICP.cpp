//
//  ICP.cpp
//  pointcloud_registration
//
//  Created by Allen Tang on 6/7/18.
//  Copyright © 2018 Allen Tang. All rights reserved.
//

#include "ICP.hpp"

void computeICPAlignment(const pcl::PointCloud<pcl::PointXYZ>::Ptr &source, const pcl::PointCloud<pcl::PointXYZ>::Ptr &target, pcl::PointCloud<pcl::PointXYZ>::Ptr &source_aligned, Eigen::Matrix4f &T, int num_iter) {
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setMaximumIterations(num_iter);
    icp.setMaxCorrespondenceDistance(CORR_DIST);
    icp.setRANSACOutlierRejectionThreshold(RANSAC_DIST);
    icp.setEuclideanFitnessEpsilon(CONV_EPS);
    icp.setInputCloud (source);
    icp.setInputTarget (target);
    icp.align (*source_aligned);
    std::cout << "has converged:" << icp.hasConverged() << " score: " <<
    icp.getFitnessScore() << std::endl;
    T = icp.getFinalTransformation();
    std::cout << T << std::endl;

}
