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
#include "keypoints.hpp"
#include "ICP.hpp"
#include "utilities.hpp"


enum keypoint_t {ISS, SIFT, HARRIS};
enum descriptor_t {ICP, FPFH, IS};

using namespace pcl;
using namespace std;

int main(int argc, const char * argv[]) {
    // Initialize arguments
    string sourceFilename = "../../../../point_cloud_registration1/pointcloud1_ned.ply";
    string targetFilename = "../../../../point_cloud_registration1/pointcloud2_ned.ply";
    string resultsDirectory = "../../../results/";
    keypoint_t keypoint = ISS;
    descriptor_t descriptor = ICP;
    
    // Parse arguments
    if (argc > 1) {
        for (int i = 1; i < argc; i++) {
            string arg = argv[i];
            if (arg == "-s") {
                sourceFilename = argv[++i];
            } else if (arg == "-t") {
                targetFilename = argv[++i];
            } else if (arg == "-r") {
                resultsDirectory = argv[++i];
            } else if (arg == "-k") {
                keypoint = (keypoint_t) *argv[++i];
            } else if (arg == "-d") {
                descriptor = (descriptor_t) *argv[++i];
            } else {
                std::cerr << "Unknown argument" << std::endl;
            }
        }
    }

    
    // Load point clouds
    PointCloud<PointXYZ>::Ptr source (new PointCloud<PointXYZ>);
    PointCloud<PointXYZ>::Ptr target (new PointCloud<PointXYZ>);
    if (io::loadPLYFile(sourceFilename, *source) == -1) {
        PCL_ERROR ("Couldn't read source file \n");
        return (-1);
    }
    
    if (io::loadPLYFile(targetFilename, *target) == -1) {
        PCL_ERROR ("Couldn't read target file \n");
        return (-1);
    }
    
    // Initialize alignment variables
    PointCloud<PointXYZ>::Ptr source_keypoints_aligned (new PointCloud<PointXYZ>);
    PointCloud<PointXYZ>::Ptr source_aligned (new PointCloud<PointXYZ>);
    Eigen::Matrix4f T;
    
    // Initialize clock
    clock_t start;
    double duration;
    
    start = clock();
    
    // Find keypoints
    PointCloud<PointXYZ>::Ptr source_keypoints (new PointCloud<PointXYZ>);
    PointCloud<PointXYZ>::Ptr target_keypoints (new PointCloud<PointXYZ>);
    switch (keypoint){
        case ISS: {
            computeISSKeypoints(source, source_keypoints);
            computeISSKeypoints(target, target_keypoints);
        }
    }
    // Save keypoints
    io::savePLYFileBinary(resultsDirectory + "source_keypoints.ply", *source_keypoints);
    io::savePLYFileBinary(resultsDirectory + "target_keypoints.ply", *target_keypoints);
    
    switch (descriptor)
    {
        case ICP: {
            // Perform ICP
            computeICPAlignment(source_keypoints, target_keypoints, source_keypoints_aligned, T);
            break;
        }
        case FPFH: {
            break;
        }
        case IS: {
            
            break;
        }
    }
    duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
    cout << "Computation time: " << duration << "\n";
    
    
    io::savePLYFileBinary(resultsDirectory + "source_keypoints_aligned.ply", *source_keypoints_aligned);
    transformPointCloud(*source, *source_aligned, T);
    io::savePLYFileBinary(resultsDirectory + "source_aligned.ply", *source_aligned);
    return 0;
}
