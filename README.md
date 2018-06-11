# Point Cloud Registration
# Dayeon Hwang and Allen Tang
# Geospatial Spring 2018

This program computes the alignment between two 3D point clouds using various key points and feature descriptors. After initial registration is computed using the specified feature descriptor, the registration is further refined using Iterative Closest Point.

Dependencies:
Point Cloud Library (pcl) v1.8.1

Arguments:
“-s”: source point cloud filepath (.ply)
“-t”: target point cloud filepath (.ply)
“-r”: results directory 
“-k”: key point type, {0: ISS, 1: HARRIS, 2: SIFT}
“-d”: descriptor type, {0: FPFH, 1: Intensity Spin, 2: RIFT, 3: NONE}

Output: 
-Aligned Source Point Clouds (.ply)
-Transformation Matrix (.txt)
-Fitness Score and Computation Time (.txt)


