# Relocalization Evaluation Code
This code computes the error between given poses of two distinct odometry trajectories. The translation and rotation errors are computed relative to the given ground truth. Also, it is possible to activate a preliminary pose refinement via (G)ICP alignment of the current point cloud to the map point cloud. Moreover, it is possible to compute the relocalization error in the absence of ground truth. In this case, the error is only computed between the two corresponding odometry poses of the two different trajectories.

### Input files
- [mandatory] Trajectories bags: they should contain the odometry, and ground truth poses to be compared.
- [mandatory] Timestamps file: it contains a list of the timestamps to identify the specific poses we want to evaluate. Each line contains a single timestamp. The timestamp should be written as double numbers, that is, using the following convention: seconds.nano_seconds. You have to list first the timestamps relative to the first trajectory, the **order matters**! Then you list the timestamps of the second trajectory in the same order. Timestamps are relative to the /odom pose topic to be evaluated.
- [optional] Trajectories bags with point cloud messages: trajectories bags with sensor point cloud messages to be used to compute the alignment with the map point cloud.
- [optional] map.pcd: the map againts which compute (G)ICP alignments.

### Output files
- reloc_error.txt: relocalization errors of each area (translation and rotation).
- statistics.txt: relocalization errors statistics (max, mean, min).
- gicp_reloc_error.txt [if (G)ICP alignment is used]: relocalization errors after (G)ICP alignment.

The output files are printed in a format to be directly copied and pasted in a LaTeX table :)

### Building instructions
```
cd relocalization_evaluation
mkdir build
cd build
cmake ..
make
```
## Usage
In order to use the code, you first need to create the ```input``` and ```output``` directories and fill them with the required files as specified above.
```
cd relocalization_evaluation
mkdir input
mkdir output
```
Once you have created and filled the directories, you need to modify the configuration parameters. You can find the configuration parameters at the beginning of the ```int main (int argc, char** argv)``` function. The parameters are:
- [mandatory] traj1_bag_name: the path of the .bag file containing the first trajectory (with both the odometry and its ground truth)
- [mandatory] traj2_bag_name: the path of the .bag file containing the second trajectory (with both the odometry and its ground truth)
- [optional] traj1_points_bag_name: the path of the .bag file containing the point cloud related to the first trajectory
- [optional] traj2_points_bag_name: the path of the .bag file containing the point cloud related to the second trajectory
- [mandatory] timestamps_file_name: the path of the ```timestamps.txt``` file, filled as specified above
- [mandatory] odometry_topic: the name of the odometry topic as in the ```traj*_bag_name``` bag
- [mandatory] ground_truth_topic: the name of the ground truth topic as in the ```traj*_bag_name``` bag; in case you want to compute the error without the ground truth, do not initialize this parameter (```const std::string ground_truth_topic;```) and set the ```no_ground_truth parameter``` to ```true```
- [optional] point_cloud_topic: the name of the point cloud topic as in the ```traj*_points_bag_name``` bag
- [optional] map_name: the path of the .pcd file containing the map onto which align the point clouds
- [mandatory] n_poses: the number of poses of a single trajectory for which you want to compute the relocalization error; this number should match the number of timestamps in the ```timestamps.txt``` file 
- [mandatory] improve_alignment: a boolean value to specify if you want to activate the (G)ICP refinement
- [mandatory] bool_visualize: a boolean value to specify if you want to visualize the results of the (G)ICP alignment
- [mandatory] no_ground_truth: a boolean value to specify if you want to use the code without a ground truth

Once you have set the parameters, you need to recompile the project.
