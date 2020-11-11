# Relocalization Evaluation Code
This code computes the error between given poses of two distinct odometry trajectories. The translation and rotation errors are computed relative to a given ground truth. Also, it is possible to activate a preliminary pose refinement via (G)ICP alignment of the current point cloud to the map point cloud. 

### Input files
- Trajectories bags [mandatory]: they should contain the odometry and ground truth poses to be compared 
- Timestamps file [mandatory]: it contains a list of the timestamps to identify the specific poses we want to evaluate. Each line contains a single timestamp. Timestamp should be written as double numbers, that is, using the following convention: seconds.nano_seconds. You have to list first the timestamps relative to the first trajectory, the order MATTERS! Then you list the timestamps of the second trajectory in the same order. Timestamps are relative to the /odom pose topic to be evaluated.
- Trajectories bags with point cloud messages [optional]: trajectories bags with sensor point cloud messages to be used to compute the alignment with the map point cloud.
- map.pcd [optional]: the map againts which compute (G)ICP alignments.

### Output files
- reloc_error.txt: relocalization errors of each area (translation and rotation)
- statistics.txt: relocalization errors statistics (max, mean, min)
- gicp_reloc_error.txt [if (G)ICP alignment is used]: relocalization errors after (G)ICP alignment


### Building instructions

cd relocalization_evaluation
mkdir build
cd build
cmake ..
make

