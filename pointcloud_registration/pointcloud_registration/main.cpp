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

//#include <pcl/point_cloud.h>
//#include <pcl/point_types.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/common/io.h>
//#include <pcl/features/normal_3d.h>
//#include <pcl/kdtree/kdtree_flann.h>
//#include <pcl/features/rift.h>
//#include <pcl/features/spin_image.h>
//#include <pcl/features/intensity_spin.h>
//#include <pcl/features/intensity_gradient.h>

enum keypoint_t {ISS, SIFT, HARRIS};
enum descriptor_t {FPFH, IS, RIFT, NONE};

using namespace pcl;
using namespace std;

int main(int argc, const char * argv[]) {
    // Initialize arguments
    string sourceFilename = "data/pointcloud1_ned.ply";
    string targetFilename = "data/pointcloud2_ned.ply";
    string resultsDirectory = "results/";
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

    PointCloud<Normal>::Ptr source_normals (new PointCloud<Normal>);
    PointCloud<Normal>::Ptr target_normals (new PointCloud<Normal>);

    // Compute normals
    computeNormals(source_keypoints, source_normals, 0.1);
    computeNormals(target_keypoints, target_normals, 0.1);

    // Perform initial alignment
    PointCloud<PointXYZI>::Ptr source_keypoints_ia (new PointCloud<PointXYZI>);
    Eigen::Matrix4f T_initial;
    double fit_score_ia;
    cout << "Performing Initial Alignment..." << endl;
    switch (descriptor)
    {
        case FPFH: {
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
            boost::shared_ptr<Correspondences> correspondences (new Correspondences);
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

            // Compute rigid transformation
            pcl::registration::TransformationEstimationSVD<PointXYZI, PointXYZI> trans_est_svd;
            trans_est_svd.estimateRigidTransformation(*source_keypoints, *target_keypoints, *correspondences, T_initial);
            transformPointCloud(*source_keypoints, *source_keypoints_ia, T_initial);
            break;
        }
        case IS: {
            // convert source_keypoints to PointXYZI
            pcl::PointCloud<PointXYZI>::Ptr cloud_xyzi (new PointCloud<PointXYZI>);
            pcl::copyPointCloud(*source_keypoints, *cloud_xyzi);

            // perform Intensity Spin Estimation
            typedef Histogram<20> IntensitySpin;
            IntensitySpinEstimation<PointXYZI, IntensitySpin> ispin_est;
            search::KdTree<PointXYZI>::Ptr treept3 (new search::KdTree<PointXYZI> (false));
            ispin_est.setSearchMethod (treept3);
            ispin_est.setRadiusSearch (10.0);
            ispin_est.setNrDistanceBins (4);
            ispin_est.setNrIntensityBins (5);

            ispin_est.setInputCloud (cloud_xyzi->makeShared ());
            PointCloud<IntensitySpin> ispin_output;
            ispin_est.compute (ispin_output);

            break;
        }
        case RIFT: {

        }
        case NONE: {
            fit_score_ia = 0.0;
            copyPointCloud(*source_keypoints, *source_keypoints_ia);
        }
    }

//    cout << "Initial Fitness Score:" << fit_score_ia << endl;
    cout << "Performing ICP Refinement..." << endl;
    // Refine with ICP
    PointCloud<PointXYZI>::Ptr source_keypoints_aligned (new PointCloud<PointXYZI>);
    Eigen::Matrix4f T_ICP;
    computeICPAlignment(source_keypoints_ia, target_keypoints, source_keypoints_aligned, T_ICP, 20000);

    duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
    cout << "Computation time: " << duration << "\n";

    // Save aligned source clouds
    io::savePLYFileBinary(resultsDirectory + "source_keypoints_ia.ply", *source_keypoints_ia);
    io::savePLYFileBinary(resultsDirectory + "source_keypoints_aligned.ply", *source_keypoints_aligned);
    PointCloud<PointXYZI>::Ptr source_aligned (new PointCloud<PointXYZI>);
    Eigen::Matrix4f T_final = T_initial * T_ICP;
    transformPointCloud(*source, *source_aligned, T_final);
    io::savePLYFileBinary(resultsDirectory + "source_aligned.ply", *source_aligned);
    return 0;
}
