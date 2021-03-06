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
enum descriptor_t {FPFH, IS, R, ICP};


using namespace pcl;
using namespace std;

int main(int argc, const char * argv[]) {
    // Initialize arguments
    string sourceFilename = "../../../../point_cloud_registration1/pointcloud1_ned.ply";
    string targetFilename = "../../../../point_cloud_registration1/pointcloud2_ned.ply";
    string resultsDirectory = "../../../results/";
    keypoint_t keypoint = HARRIS;
    descriptor_t descriptor = ICP;
    string key_name;
    string des_name;
    bool load_keypoints = false;

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
            if (!load_keypoints) {
                computeISSKeypoints(source, source_keypoints);
                computeISSKeypoints(target, target_keypoints);
            }
            break;
        }
        case SIFT: {
            key_name = "SIFT";
            if (!load_keypoints) {
                computeSIFTKeypoints(source, source_keypoints);
                computeSIFTKeypoints(target, target_keypoints);
            }
            break;
            ;
        }
        case HARRIS: {
            key_name = "HARRIS";
            if (!load_keypoints) {
                computeHARRISKeypoints(source, source_keypoints);
                computeHARRISKeypoints(target, target_keypoints);
            }
            break;
        }
    }
    
    if (!load_keypoints) {
        // Save keypoints
        io::savePLYFileBinary(resultsDirectory + "source_keypoints_" + key_name + ".ply", *source_keypoints);
        io::savePLYFileBinary(resultsDirectory + "target_keypoints_" + key_name + ".ply", *target_keypoints);
    } else {
        // Load keypoints
        if (io::loadPLYFile(resultsDirectory + "source_keypoints_" + key_name + ".ply", *source_keypoints) == -1) {
            PCL_ERROR ("Couldn't read source keypoints \n");
            return (-1);
        }
        if (io::loadPLYFile(resultsDirectory + "target_keypoints_" + key_name + ".ply", *target_keypoints) == -1) {
            PCL_ERROR ("Couldn't read target keypoints \n");
            return (-1);
            }
    }


    PointCloud<Normal>::Ptr source_normals (new PointCloud<Normal>);
    PointCloud<Normal>::Ptr target_normals (new PointCloud<Normal>);

    // Compute normals
    computeNormals(source_keypoints, source_normals, 0.1);
    computeNormals(target_keypoints, target_normals, 0.1);

    // Perform initial alignment
    PointCloud<PointXYZI>::Ptr source_keypoints_aligned (new PointCloud<PointXYZI>);
    Eigen::Matrix4f T;
    double score;
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

            // Perform correspondence estimation
            registration::CorrespondenceEstimation<FPFHSignature33, FPFHSignature33> corr_est;
            search::KdTree<FPFHSignature33>::Ptr corr_tree (new search::KdTree<FPFHSignature33>);
            corr_est.setSearchMethodTarget(corr_tree);
            corr_est.setInputSource (source_features);
            corr_est.setInputTarget (target_features);
            corr_est.determineReciprocalCorrespondences (*correspondences);

            break;
        }
        case IS: {
            des_name = "IS";
            // perform Intensity Spin Estimation

            pcl::PointCloud<IntensitySpin>::Ptr source_features (new PointCloud<IntensitySpin>);
            pcl::PointCloud<IntensitySpin>::Ptr target_features (new PointCloud<IntensitySpin>);
            computeISFeatures(source_keypoints, source_features);
            computeISFeatures(target_keypoints, target_features);


            // Perform correspondence estimation
            registration::CorrespondenceEstimation<IntensitySpin, IntensitySpin> corr_est;
            search::KdTree<IntensitySpin>::Ptr corr_tree (new search::KdTree<IntensitySpin>);
            corr_est.setSearchMethodTarget(corr_tree);
            corr_est.setInputSource (source_features);
            corr_est.setInputTarget (target_features);
            corr_est.determineReciprocalCorrespondences (*correspondences);

            break;
        }
        case R: {
            des_name = "R";
            pcl::PointCloud<RIFT>::Ptr source_features (new PointCloud<RIFT>);
            pcl::PointCloud<RIFT>::Ptr target_features (new PointCloud<RIFT>);
            computeRIFTFeatures(source_keypoints, source_features);
            computeRIFTFeatures(target_keypoints, target_features);

            // Perform correspondence estimation
            registration::CorrespondenceEstimation<RIFT, RIFT> corr_est;
            search::KdTree<RIFT>::Ptr corr_tree (new search::KdTree<RIFT>);
            corr_est.setSearchMethodTarget(corr_tree);
            corr_est.setInputSource (source_features);
            corr_est.setInputTarget (target_features);
            corr_est.determineReciprocalCorrespondences (*correspondences);

            break;
        }
        case ICP: {
            des_name = "ICP";
        }
    }

    boost::shared_ptr<pcl::Correspondences> correspondences_final (new pcl::Correspondences);

    if (descriptor == ICP) {
        int ICP_iter = 50000;
        double score_final = computeICPAlignment(source_keypoints, target_keypoints, source_keypoints_aligned, T, ICP_iter);
    } else {
    
        //Perform correspondence rejection using sample consensus
        pcl::registration::CorrespondenceRejectorSampleConsensus<PointXYZI> corr_rej_sac;
        corr_rej_sac.setInputSource (source_keypoints);
        corr_rej_sac.setInputTarget (target_keypoints);
        corr_rej_sac.setInlierThreshold (0.05);
        corr_rej_sac.setMaximumIterations (10000);
        corr_rej_sac.setInputCorrespondences (correspondences);
        corr_rej_sac.getCorrespondences (*correspondences_final);

        // Compute rigid transformation
        registration::TransformationEstimationSVD<PointXYZI, PointXYZI> trans_est_svd;
        trans_est_svd.estimateRigidTransformation(*source_keypoints, *target_keypoints, *correspondences_final, T);
        transformPointCloud(*source_keypoints, *source_keypoints_aligned, T);
//        score_test = computeFitScore(source_keypoints_ia, target_keypoints, correspondences_final);
    }

    duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
    cout << "Computation time: " << duration << "\n";

    // Save aligned source clouds
    io::savePLYFileBinary(resultsDirectory + "source_keypoints_aligned_" + key_name + "_" + des_name + ".ply", *source_keypoints_aligned);
    PointCloud<PointXYZI>::Ptr source_aligned (new PointCloud<PointXYZI>);
    transformPointCloud(*source, *source_aligned, T);
    io::savePLYFileBinary(resultsDirectory + "source_aligned_" + key_name + "_" + des_name + ".ply", *source_aligned);

    // Compute L2SQR score
    registration::TransformationValidationEuclidean<pcl::PointXYZI, pcl::PointXYZI> tve;
    tve.setMaxRange (0.1);  // 1cm
    score = tve.validateTransformation (source, target, T);
    
//    registration::TransformationValidationEuclidean<pcl::PointXYZI, pcl::PointXYZI> tve_final;
//    tve_final.setMaxRange (0.1);  // 1cm
//    score_final = tve.validateTransformation (source, target, T_final);

    // Store data in text file
    ofstream alignInfo;
    alignInfo.open(resultsDirectory + "results_" + key_name + "_" + des_name + ".txt");
    alignInfo << "Keypoint type: " << key_name << endl;
    alignInfo << "Descriptor type: " << des_name << endl;
//    alignInfo << "Initial alignment score: " << score_ia << endl;
//    alignInfo << "Initial Tranformation Matrix: \n" << std::setprecision(20) << T_initial << endl;
//    alignInfo << "ICP alignment score after " << ICP_iter << "iterations: " << score_final << endl;
//    alignInfo << "Final Tranformation Matrix: \n" << std::setprecision(20) << T_final << endl;
    alignInfo << "Alignment score: " << score << endl;
    alignInfo << "Transformation Matrix: " << setprecision(20) << T << endl;
    alignInfo << "Total computation time: " << duration << " seconds." << endl;
    alignInfo.close();

    return 0;
}
