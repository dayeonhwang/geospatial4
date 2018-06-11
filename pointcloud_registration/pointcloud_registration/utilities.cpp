//
//  utilities.cpp
//  pointcloud_registration
//
//  Created by Allen Tang on 6/7/18.
//  Copyright © 2018 Allen Tang. All rights reserved.
//

#include "utilities.hpp"


double computeFitScore (const pcl::PointCloud<pcl::PointXYZI>::Ptr &source, const pcl::PointCloud<pcl::PointXYZI>::Ptr &target, const boost::shared_ptr<pcl::Correspondences> &correspondences) {
    double sum = 0.0;
    for (int i = 0; i < correspondences->size(); i++) {
        pcl::PointXYZI query_pt = (*source)[(*correspondences)[i].index_query];
        pcl::PointXYZI match_pt = (*target)[(*correspondences)[i].index_match];
        sum += (double) pcl::geometry::squaredDistance(query_pt, match_pt);
    }
    return sum;
}
