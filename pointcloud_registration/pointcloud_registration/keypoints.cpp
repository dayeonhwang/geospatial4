//
//  keypoints.cpp
//  pointcloud_registration
//
//  Created by Allen Tang on 6/7/18.
//  Copyright © 2018 Allen Tang. All rights reserved.
//

#include "keypoints.hpp"

// Taken from pcl/doc/tutorials/content/sources/correspondence_grouping/correspondence_grouping.cpp
double computeCloudResolution (const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &cloud) {
    double res = 0.0;
    int n_points = 0;
    int nres;
    std::vector<int> indices (2);
    std::vector<float> sqr_distances (2);
    pcl::search::KdTree<pcl::PointXYZI> tree;
    tree.setInputCloud (cloud);
    
    for (size_t i = 0; i < cloud->size (); ++i)
    {
        if (! pcl_isfinite ((*cloud)[i].x))
        {
            continue;
        }
        //Considering the second neighbor since the first is the point itself.
        nres = tree.nearestKSearch (i, 2, indices, sqr_distances);
        if (nres == 2)
        {
            res += sqrt (sqr_distances[1]);
            ++n_points;
        }
    }
    if (n_points != 0)
    {
        res /= n_points;
    }
    return res;
}


void computeISSKeypoints(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr &keypoints) {
    double model_res = computeCloudResolution(cloud);
    double salient_radius = 6 * model_res; //6
    double nms_radius = 4 *  model_res; //4
    double gamma_21 = 0.975;
    double gamma_32 = 0.975;
    double min_neighbors = 5;
    
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
    
    pcl::ISSKeypoint3D<pcl::PointXYZI, pcl::PointXYZI> iss_detector;
    iss_detector.setSearchMethod (tree);
    iss_detector.setSalientRadius (salient_radius);
    iss_detector.setNonMaxRadius (nms_radius);
    iss_detector.setThreshold21 (gamma_21);
    iss_detector.setThreshold32 (gamma_32);
    iss_detector.setMinNeighbors (min_neighbors);
    iss_detector.setInputCloud (cloud);
    iss_detector.compute (*keypoints);
    
}
