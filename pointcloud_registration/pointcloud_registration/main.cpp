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
#include "descriptors.hpp"


enum keypoint_t {ISS, SIFT, HARRIS};
enum descriptor_t {FPFH, IS, NONE};

using namespace pcl;
using namespace std;

int main(int argc, const char * argv[]) {
    // Initialize arguments
    string sourceFilename = "../../../../point_cloud_registration1/pointcloud1_ned.ply";
    string targetFilename = "../../../../point_cloud_registration1/pointcloud2_ned.ply";
    string resultsDirectory = "../../../results/";
    keypoint_t keypoint = ISS;
    descriptor_t descriptor = FPFH;
    
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
        case SIFT: {
            ;
        }
        case HARRIS: {
            ;
        }
    }
    // Save keypoints
    io::savePLYFileBinary(resultsDirectory + "source_keypoints.ply", *source_keypoints);
    io::savePLYFileBinary(resultsDirectory + "target_keypoints.ply", *target_keypoints);
    
    pcl::PointCloud<pcl::Normal> source_normals;
    pcl::PointCloud<pcl::Normal> target_normals;
    
    // Compute normals
    computeNormals(source_keypoints, source_normals);
    computeNormals(target_keypoints, target_normals);
    
    // Perform initial alignment
    PointCloud<PointXYZ>::Ptr source_keypoints_ia (new PointCloud<PointXYZ>);
    Eigen::Matrix4f T_initial;
    double fit_score_ia;
    cout << "Performing Initial Alignment..." << endl;
    switch (descriptor)
    {
        case FPFH: {
            pcl::PointCloud<pcl::FPFHSignature33> source_features, target_features;
            computeFPFHFeatures(source_keypoints, source_normals, source_features);
            computeFPFHFeatures(target_keypoints, target_normals, target_features);
            
            SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> reg;
            reg.setMinSampleDistance (0.05f);
            reg.setMaxCorrespondenceDistance (0.05);
            reg.setMaximumIterations (20000);
            pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZ>::Ptr rej_samp (new pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZ>);
            reg.addCorrespondenceRejector(rej_samp);
            reg.setEuclideanFitnessEpsilon(0.0001);
            reg.setInputCloud (source_keypoints);
            reg.setInputTarget (target_keypoints);
            reg.setSourceFeatures (source_features.makeShared());
            reg.setTargetFeatures (target_features.makeShared());
            reg.align (*source_keypoints_ia);
            T_initial = reg.getFinalTransformation();
            fit_score_ia = reg.getFitnessScore();
            
            break;
        }
        case IS: {
            
            break;
        }
        case NONE: {
            fit_score_ia = 0.0;
            copyPointCloud(*source_keypoints, *source_keypoints_ia);
        }
    }
    
    cout << "Initial Fitness Score:" << fit_score_ia << endl;
    cout << "Performing ICP Refinement..." << endl;
    // Refine with ICP
    PointCloud<PointXYZ>::Ptr source_keypoints_aligned (new PointCloud<PointXYZ>);
    Eigen::Matrix4f T_ICP;
    computeICPAlignment(source_keypoints_ia, target_keypoints, source_keypoints_aligned, T_ICP, 20000);
    
    duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
    cout << "Computation time: " << duration << "\n";
    
    // Save aligned source clouds
    io::savePLYFileBinary(resultsDirectory + "source_keypoints_aligned.ply", *source_keypoints_aligned);
    PointCloud<PointXYZ>::Ptr source_aligned (new PointCloud<PointXYZ>);
    Eigen::Matrix4f T_final = T_initial * T_ICP;
    transformPointCloud(*source, *source_aligned, T_final);
    io::savePLYFileBinary(resultsDirectory + "source_aligned.ply", *source_aligned);
    return 0;
}
