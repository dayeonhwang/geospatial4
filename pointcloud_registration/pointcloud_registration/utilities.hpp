//
//  utilities.hpp
//  pointcloud_registration
//
//  Created by Allen Tang on 6/7/18.
//  Copyright © 2018 Allen Tang. All rights reserved.
//

#ifndef utilities_hpp
#define utilities_hpp

//#define PCL_NO_PRECOMPILE
#include <stdio.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/registration.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/ppf.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/rift.h>
#include <pcl/features/spin_image.h>
#include <pcl/features/intensity_spin.h>
#include <pcl/features/intensity_gradient.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/transformation_validation_euclidean.h>
#include <pcl/common/geometry.h>

#include <Eigen/Core>
#include <Eigen/Dense>

double computeFitScore (const pcl::PointCloud<pcl::PointXYZI>::Ptr &source, const pcl::PointCloud<pcl::PointXYZI>::Ptr &target, const boost::shared_ptr<pcl::Correspondences> &correspondences);


#endif /* utilities_hpp */
