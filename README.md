### Input files

- Trajectories bags: they should contain the odometry and ground truth poses to be compared 
- Timestamps file: it contains a list of the timestamps to identify the specific poses we want to evaluate. Each line contains a single timestamp. Timestamp should be written as double numbers, that is, using the following convention: seconds.nano_seconds. You have to list first the timestamps relative to the first trajectory, the order MATTERS! Then you list the timestamps of the second trajectory in the same order. Timestamps are relative to the /odom pose topic to be evaluated.
- Trajectories bags with point cloud messages: trajectories bags with sensor point cloud messages to be used to compute the alignment with the map point cloud.
- map.pcd: the map againts which compute (G)ICP alignments.

### Output files


### Building instructions

cd relocalization_evaluation
mkdir build
cd build
cmake ..
make

