import pymap3d as pm
import numpy as np
import sys, io, os

if __name__ == '__main__':
    arg = sys.argv[:]
    if len(arg) < 3:
        print("Not enough arguments\n")
        exit(0)
    directory = str(arg[1])
    filename = str(arg[2])
    fullpath = directory + filename
    # Read data into numpy array
    fuse = open(fullpath, 'rb')
    data = np.genfromtxt(fuse, delimiter=' ')
    num_points = np.shape(data)[0]
    # Create new file and add ply header
    ply = open(directory + filename.split('.')[0] + '_ned.ply', 'w+')
    header = 'ply\nformat ascii 1.0\nelement vertex ' + str(num_points) \
        + '\nproperty float x\nproperty float y\nproperty float z\n' \
        + 'property uchar intensity\nelement face 0\n' \
        + 'property list uchar int vertex_indices\nend_header\n'
    ply.write(header)
    # Convert each point from WGS48 to NED and store in file
    lat0 = 48.858858
    lon0 = 2.299525
    h0 = 76.995310
    for i in range(num_points):
        point = data[i]
        x, y, z = pm.geodetic2ned(point[0], point[1], point[2], lat0, lon0, h0)
        # x, y , z = pm.geodetic2ecef(point[0], point[1], point[2])
        row = str(x) + ' ' + str(y) + ' ' + str(z) + ' ' + str(int(point[3])) + '\n'
        ply.write(row)
    ply.close()
    fuse.close()
