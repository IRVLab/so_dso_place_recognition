# We use SO-DSO as stereo visual odometry to generate 3D points and poses
Check [SO-DSO](https://github.com/jiawei-mo/scale_optimization) for details.

# Files for place recognition in .ros folder
- poses_history_file.txt: [incoming_id, pose[t11, t12, ..., t34]]
- pts_history_file.txt: [incoming_id, location in local frame[x,y,z], color intensity]
