# Place recognition using 3D points from SO-DSO.

# Usage
- Run SO-DSO to generate 3D points and poses, see [instruction](https://github.com/jiawei-mo/3d_place_recognition/blob/master/so_dso/README.md)
- Generate descriptors, see examples in launch folder
- Run place recognition, see ./matlab/pair/robotcar.m

# Code Explaination
- utils/pts_preprocess: accumulates points locally, extracts a spherical point cloud by pose. 
- utils/pts_align: aligns the point cloud by PCA.
- src/X/X: X algorithm to describe the filtered spherical point cloud.
- src/X/test_X: calls the above functions.
- matlab/single|pair: recognizes places according to the descriptors.