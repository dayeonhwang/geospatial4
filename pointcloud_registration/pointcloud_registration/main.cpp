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



enum method_t {ICP, ISS, FPFH};

using namespace pcl;
using namespace std;

int main(int argc, const char * argv[]) {
    // Initialize arguments
    string sourceFilename = "../../../../point_cloud_registration1/pointcloud1.ply";
    string targetFilename = "../../../../point_cloud_registration1/pointcloud2.ply";
    string resultsDirectory = "../../../results/";
    method_t method = ICP;
    int method_int;
    
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
            } else if (arg == "-m") {
                method = (method_t) *argv[++i];
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
    
    PointCloud<PointXYZ>::Ptr source_aligned (new PointCloud<PointXYZ>);
    
    // Initialize clock
    clock_t start;
    double duration;
    
    start = clock();
    switch (method)
    {
        case ICP: {
            // Find keypoints
            PointCloud<PointXYZ>::Ptr source_keypoints (new PointCloud<PointXYZ>);
            PointCloud<PointXYZ>::Ptr target_keypoints (new PointCloud<PointXYZ>);
            computeISSKeypoints(source, source_keypoints);
            computeISSKeypoints(target, target_keypoints);
            
            // Perform ICP
            Eigen::Matrix4f T;
            computeICPAlignment(source_keypoints, target_keypoints, source_aligned, T);
            break;
        }
        case ISS: {
            break;
        }
        case FPFH: {
            break;
        }
    }
    duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
    cout << "Computation time: " << duration << "\n";
    io::savePLYFileBinary(resultsDirectory + "source_aligned.ply", *source_aligned);
    
    return 0;
}
