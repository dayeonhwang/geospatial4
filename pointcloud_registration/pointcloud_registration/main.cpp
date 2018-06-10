;//
//  main.cpp
//  pointcloud_registration
//
//  Created by Allen Tang on 6/4/18.
//  Copyright © 2018 Allen Tang. All rights reserved.
//

#define PCL_NO_PRECOMPILE
#include <iostream>
#include <stdio.h>
#include <fstream>
#include <iomanip>
#include "keypoints.hpp"
#include "ICP.hpp"
#include "utilities.hpp"
#include "descriptors.hpp"



enum keypoint_t {ISS, SIFT, HARRIS};
enum descriptor_t {FPFH, IS, RIFT, NONE};


using namespace pcl;
using namespace std;

int main(int argc, const char * argv[]) {
    // Initialize arguments
    string sourceFilename = "../../../../point_cloud_registration1/pointcloud1_ned.ply";
    string targetFilename = "../../../../point_cloud_registration1/pointcloud2_ned.ply";
    string resultsDirectory = "../../../results/";
    keypoint_t keypoint = ISS;
    descriptor_t descriptor = NONE;
    string key_name;
    string des_name;
    
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
    PointCloud<PointXYZI>::Ptr source (new PointCloud<PointXYZI>);
    PointCloud<PointXYZI>::Ptr target (new PointCloud<PointXYZI>);
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
    PointCloud<PointXYZI>::Ptr source_keypoints (new PointCloud<PointXYZI>);
    PointCloud<PointXYZI>::Ptr target_keypoints (new PointCloud<PointXYZI>);
    switch (keypoint){
        case ISS: {
            key_name = "ISS";
            computeISSKeypoints(source, source_keypoints);
            computeISSKeypoints(target, target_keypoints);
            break;
        }
        case SIFT: {
            key_name = "SIFT";
            computeSIFTKeypoints(source, source_keypoints);
            computeSIFTKeypoints(target, target_keypoints);
            break;
            ;
        }
        case HARRIS: {
            key_name = "HARRIS";
            computeHARRISKeypoints(source, source_keypoints);
            computeHARRISKeypoints(target, target_keypoints);
            break;
        }
    }
    // Save keypoints
    io::savePLYFileBinary(resultsDirectory + "source_keypoints_" + key_name + ".ply", *source_keypoints);
    io::savePLYFileBinary(resultsDirectory + "target_keypoints_" + key_name + ".ply", *target_keypoints);

    PointCloud<Normal>::Ptr source_normals (new PointCloud<Normal>);
    PointCloud<Normal>::Ptr target_normals (new PointCloud<Normal>);

    // Compute normals
    computeNormals(source_keypoints, source_normals, 0.1);
    computeNormals(target_keypoints, target_normals, 0.1);

    // Perform initial alignment
    PointCloud<PointXYZI>::Ptr source_keypoints_ia (new PointCloud<PointXYZI>);
    Eigen::Matrix4f T_initial;
    double fit_score_ia;
    boost::shared_ptr<Correspondences> correspondences (new Correspondences);
    cout << "Performing Initial Alignment..." << endl;
    switch (descriptor)
    {
        case FPFH: {
            des_name = "FPFH";
            pcl::PointCloud<FPFHSignature33>::Ptr source_features (new PointCloud<FPFHSignature33>);
            pcl::PointCloud<FPFHSignature33>::Ptr target_features (new PointCloud<FPFHSignature33>);
            computeFPFHFeatures(source_keypoints, source_normals, source_features, 0.1);
            computeFPFHFeatures(target_keypoints, target_normals, target_features, 0.1);

//            SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> reg;
//            reg.setMinSampleDistance (0.05f);
//            reg.setMaxCorrespondenceDistance (0.1);
//            reg.setMaximumIterations (5000);
//            pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZ>::Ptr rej_samp (new pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZ>);
//            reg.addCorrespondenceRejector(rej_samp);
//            reg.setEuclideanFitnessEpsilon(0.0001);
//            reg.setInputCloud (source_keypoints);
//            reg.setInputTarget (target_keypoints);
//            reg.setSourceFeatures (source_features.makeShared());
//            reg.setTargetFeatures (target_features.makeShared());
//            reg.align (*source_keypoints_ia);
//            T_initial = reg.getFinalTransformation();
//            fit_score_ia = reg.getFitnessScore();

            // Perform correspondence estimation
            registration::CorrespondenceEstimation<FPFHSignature33, FPFHSignature33> corr_est;
            search::KdTree<FPFHSignature33>::Ptr corr_tree (new search::KdTree<FPFHSignature33>);
            corr_est.setSearchMethodTarget(corr_tree);
            corr_est.setInputSource (source_features);
            corr_est.setInputTarget (target_features);
            corr_est.determineReciprocalCorrespondences (*correspondences);

            // Perform correspondence rejection
//            boost::shared_ptr<pcl::Correspondences> correspondences_final (new pcl::Correspondences);
//            pcl::registration::CorrespondenceRejectorSampleConsensus<FPFHSignature33> corr_rej_sac;
//            corr_rej_sac.setInputCloud (source_features);
//            corr_rej_sac.setTargetCloud (target_features);
//            corr_rej_sac.setInlierThreshold (0.01);
//            corr_rej_sac.setMaxIterations (5000);
//            corr_rej_sac.setInputCorrespondences (correspondences);
//            corr_rej_sac.getCorrespondences (*correspondences_final);

            break;
        }
        case IS: {
            des_name = "IS";
            // perform Intensity Spin Estimation

            pcl::PointCloud<IntensitySpin>::Ptr source_features (new PointCloud<IntensitySpin>);
            pcl::PointCloud<IntensitySpin>::Ptr target_features (new PointCloud<IntensitySpin>);
//            computeISFeatures(source_keypoints, source_features);
//            computeISFeatures(target_keypoints, target_features);


            pcl::IntensitySpinEstimation<pcl::PointXYZI, IntensitySpin> ispin_est;
            pcl::search::KdTree<pcl::PointXYZI>::Ptr treept3 (new pcl::search::KdTree<pcl::PointXYZI> (false));
            ispin_est.setSearchMethod (treept3);
//            ispin_est.setKSearch(4);
            ispin_est.setRadiusSearch (1.0);
            ispin_est.setNrDistanceBins (4);
            ispin_est.setNrIntensityBins (5);
            
            ispin_est.setInputCloud (source_keypoints->makeShared ());
            ispin_est.compute (*source_features);
            
            
            ispin_est.setInputCloud (target_keypoints->makeShared());
            ispin_est.compute (*target_features);
            
            // Perform correspondence estimation
            registration::CorrespondenceEstimation<IntensitySpin, IntensitySpin> corr_est;
            search::KdTree<IntensitySpin>::Ptr corr_tree (new search::KdTree<IntensitySpin>);
            corr_est.setSearchMethodTarget(corr_tree);
            corr_est.setInputSource (source_features);
            corr_est.setInputTarget (target_features);
            corr_est.determineReciprocalCorrespondences (*correspondences);
            
            break;
        }
        case RIFT: {
            des_name = "RIFT";
        }
        case NONE: {
            des_name = "ICP";
            fit_score_ia = 0.0;
            copyPointCloud(*source_keypoints, *source_keypoints_ia);
        }
    }

    if (descriptor != NONE) {
        // Compute rigid transformation
        pcl::registration::TransformationEstimationSVD<PointXYZI, PointXYZI> trans_est_svd;
        trans_est_svd.estimateRigidTransformation(*source_keypoints, *target_keypoints, *correspondences, T_initial);
        transformPointCloud(*source_keypoints, *source_keypoints_ia, T_initial);
        
    }

//    cout << "Initial Fitness Score:" << fit_score_ia << endl;
    cout << "Performing ICP Refinement..." << endl;
    // Refine with ICP
    PointCloud<PointXYZI>::Ptr source_keypoints_aligned (new PointCloud<PointXYZI>);
    Eigen::Matrix4f T_ICP;
    computeICPAlignment(source_keypoints_ia, target_keypoints, source_keypoints_aligned, T_ICP, 50000);

    duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
    cout << "Computation time: " << duration << "\n";

    // Save aligned source clouds
    io::savePLYFileBinary(resultsDirectory + "source_keypoints_ia_" + key_name + "_" + des_name + ".ply", *source_keypoints_ia);
    io::savePLYFileBinary(resultsDirectory + "source_keypoints_aligned_" + key_name + "_" + des_name + ".ply", *source_keypoints_aligned);
    PointCloud<PointXYZI>::Ptr source_aligned (new PointCloud<PointXYZI>);
    Eigen::Matrix4f T_final = T_initial * T_ICP;
    transformPointCloud(*source, *source_aligned, T_final);
    io::savePLYFileBinary(resultsDirectory + "source_aligned_" + key_name + "_" + des_name + ".ply", *source_aligned);
    return 0;
}
