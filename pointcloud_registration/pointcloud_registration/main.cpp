//
//  main.cpp
//  pointcloud_registration
//
//  Created by Allen Tang on 6/4/18.
//  Copyright © 2018 Allen Tang. All rights reserved.
//

#include <iostream>
#include <stdio.h>
#include <fstream>
#include <iomanip>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/registration.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>

#define ITER 500000
#define CORR_DIST 0.5
#define RANSAC_DIST 0.5
#define CONV_EPS 0.0001


using namespace pcl;
using namespace std;

int main(int argc, const char * argv[]) {
    // Define directory
    string resultsDirectory = "../../../results/";
    
    
    // Load point clouds
    PointCloud<PointXYZ>::Ptr source (new PointCloud<PointXYZ>);
    PointCloud<PointXYZ>::Ptr target (new PointCloud<PointXYZ>);
    
    // Perform Initial Alignment
    
    // Perform ICP
    IterativeClosestPoint<PointXYZ, PointXYZ> icp;
    icp.setMaximumIterations(ITER);
    icp.setMaxCorrespondenceDistance(CORR_DIST);
    icp.setRANSACOutlierRejectionThreshold(RANSAC_DIST);
    icp.setEuclideanFitnessEpsilon(CONV_EPS);
    icp.setInputCloud (source);
    icp.setInputTarget (target);
    PointCloud<PointXYZ>::Ptr source_aligned (new PointCloud<PointXYZ>);
    icp.align (*source_aligned);
    cout << "has converged:" << icp.hasConverged() << " score: " <<
    icp.getFitnessScore() << std::endl;
    Eigen::Matrix4f T = icp.getFinalTransformation();
    cout << T << endl;
    io::savePLYFileBinary(resultsDirectory + "source_aligned.ply", *source_aligned);

    return 0;
}
