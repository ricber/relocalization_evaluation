#include <iostream>
#include <vector>
#include <cmath>
#include <fstream>
#include <string> 
#include <sstream>
#include <boost/foreach.hpp>
#include <cmath>

#include <pcl/registration/gicp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>

    
typedef pcl::PointXYZ PointT;


namespace patch
{
    template < typename T > std::string to_string( const T& n )
    {
        std::ostringstream stm ;
        stm << n ;
        return stm.str() ;
    }
}


void visualize (pcl::PointCloud<PointT>::Ptr source_cloud, pcl::PointCloud<PointT>::Ptr cloud_transformed);
const static Eigen::IOFormat CSVFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "\n");
void writeToCSVfile(std::string name, Eigen::Matrix4f matrix);
void find_pose_quat(const std::string& traj_bag_name, const std::vector<double>& timestamps, const std::string& odometry_topic, const std::string& ground_truth_topic, std::vector<std::vector<double> >& poseO, std::vector<std::vector<double> >& quatO, std::vector<std::vector<double> >& poseG, std::vector<std::vector<double> >& quatG);
void print_poses(std::vector<std::vector<double> >& poseO, std::vector<std::vector<double> >& quatO, std::vector<std::vector<double> >& poseG, std::vector<std::vector<double> >& quatG);
Eigen::Matrix4d relocalization_error(std::vector<double>& poseO1, std::vector<double>& quatO1, std::vector<double>& poseO2, std::vector<double>& quatO2, std::vector<double>& poseG1, std::vector<double>& quatG1, std::vector<double>& poseG2, std::vector<double>& quatG2);
void alignment_relocalization_error(const std::string& map_name, const std::string& traj_bag_name, const std::string& point_cloud_topic, const std::vector<double> timestamps, const bool bool_visualize, std::vector<std::vector<double> >& pose, std::vector<std::vector<double> >& quat, std::vector<std::vector<double> >& poseAligned, std::vector<std::vector<double> >& quatAligned);
void print_relocalization_error(const std::string& file_name, const unsigned int n_poses, std::vector<std::vector<double> >& poseO1, std::vector<std::vector<double> >& quatO1, std::vector<std::vector<double> >& poseO2, std::vector<std::vector<double> >& quatO2, std::vector<std::vector<double> >& poseG1, std::vector<std::vector<double> >& quatG1, std::vector<std::vector<double> >& poseG2, std::vector<std::vector<double> >& quatG2);


inline double rad2deg(double angle)
{
    return angle * (180.0/3.141592653589793238463);
}


int main (int argc, char** argv)
{    
    const std::string traj1_bag_name = "../input/2020-06-17-traj1-benchmarking.bag"; // bags containing the odometry and ground truth
    const std::string traj2_bag_name = "../input/2020-06-17-traj2-benchmarking.bag";
    const std::string traj1_points_bag_name = "../input/2020-03-18-17-05-05-traj1-ouster-realsense.bag"; // bags containing the sensor's pointclouds
    const std::string traj2_points_bag_name = "../input/2020-03-18-17-10-55-traj2-ouster-realsense.bag";
    const std::string timestamps_file_name = "../input/timestamps.txt"; // selected poses
    const std::string odometry_topic = "/odom"; 
    const std::string ground_truth_topic = "/Robot_1/pose";
    const std::string point_cloud_topic = "/os1_cloud_node/points";
    const std::string map_name = "../input/map.pcd";
    const int n_poses = 10;
    const bool improve_alignment = false;
    const bool bool_visualize = false;
    const bool no_ground_truth = false;

    
    // -------- READ POSES, QUATERNIONS, AND TIMESTAMPS FROM FILES --------
    
    // declare poses and quaternions vectors
    std::vector<std::vector<double> > poseO1(n_poses, std::vector<double>(3));
    std::vector<std::vector<double> > quatO1(n_poses, std::vector<double>(4));
    std::vector<std::vector<double> > poseG1(n_poses, std::vector<double>(3));
    std::vector<std::vector<double> > quatG1(n_poses, std::vector<double>(4));
    std::vector<std::vector<double> > poseO2(n_poses, std::vector<double>(3));
    std::vector<std::vector<double> > quatO2(n_poses, std::vector<double>(4));
    std::vector<std::vector<double> > poseG2(n_poses, std::vector<double>(3));
    std::vector<std::vector<double> > quatG2(n_poses, std::vector<double>(4));
    
    // read the timestamps
    std::ifstream ifile(timestamps_file_name.c_str(), std::ios::in);
    //check to see that the file was opened correctly
    if (!ifile.is_open()) {
        std::cerr << "There was a problem opening " + timestamps_file_name << std::endl;
        exit(1);//exit or do additional error checking
    }
    double num = 0.0;
    std::vector<double> temp1;
    std::vector<double> temp2;
    int i_temp = 0;
    //keep storing values from the text file so long as data exists
    while (ifile >> num) {
        if (i_temp >= 0 && i_temp <= n_poses-1)
            temp1.push_back(num);
        else
            temp2.push_back(num);
        ++i_temp;
    }
    ifile.close();
    if (i_temp != 2*n_poses) {
        std::cerr << "The number of timestamps should be twice the number of poses" << std::endl;
        exit(1);//exit or do additional error checking
    }
    const std::vector<double> timestamps_1(temp1.begin(), temp1.end());
    const std::vector<double> timestamps_2(temp2.begin(), temp2.end());
    
    // read the odometry and groundtruth poses and quaternions from the bag
    find_pose_quat(traj1_bag_name, timestamps_1, odometry_topic, ground_truth_topic, poseO1, quatO1, poseG1, quatG1);
    find_pose_quat(traj2_bag_name, timestamps_2, odometry_topic, ground_truth_topic, poseO2, quatO2, poseG2, quatG2);
    std::cout << "----  TRAJECTORY 1  ----" << std::endl;
    print_poses(poseO1, quatO1, poseG1, quatG1);
    std::cout << "----  TRAJECTORY 2  ----" << std::endl;
    print_poses(poseO2, quatO2, poseG2, quatG2);
    
    
    // -------- COMPUTE RELOCALIZATION ERROR --------

    std::cout << "\n----  RELOCALIZATION ERROR  ----\n";
    print_relocalization_error("../output/reloc_error.txt", n_poses, poseO1, quatO1, poseO2, quatO2, poseG1, quatG1, poseG2, quatG2);
    
    
    // -------- COMPUTE RELOCALIZATION ERROR WITH IMPROVED ALIGNMENT --------
    
    if (improve_alignment)
    {
        std::cout << "\n----  RELOCALIZATION ERROR WITH IMPROVED ALIGNMENT  ----\n";
        std::vector<std::vector<double> > poseO1Aligned(n_poses, std::vector<double>(3));
        std::vector<std::vector<double> > quatO1Aligned(n_poses, std::vector<double>(4));
        alignment_relocalization_error(map_name, traj1_points_bag_name, point_cloud_topic, timestamps_1, bool_visualize, poseO1, quatO1, poseO1Aligned, quatO1Aligned);
        
        std::vector<std::vector<double> > poseO2Aligned(n_poses, std::vector<double>(3));
        std::vector<std::vector<double> > quatO2Aligned(n_poses, std::vector<double>(4));
        alignment_relocalization_error(map_name, traj2_points_bag_name, point_cloud_topic, timestamps_2, bool_visualize, poseO2, quatO2, poseO2Aligned, quatO2Aligned);
        
        print_relocalization_error("../output/gicp_reloc_error.txt", n_poses, poseO1Aligned, quatO1Aligned, poseO2Aligned, quatO2Aligned, poseG1, quatG1, poseG2, quatG2);
    }

    return (0);
}



void print_relocalization_error(const std::string& file_name, const unsigned int n_poses, std::vector<std::vector<double> >& poseO1, std::vector<std::vector<double> >& quatO1, std::vector<std::vector<double> >& poseO2, std::vector<std::vector<double> >& quatO2, std::vector<std::vector<double> >& poseG1, std::vector<std::vector<double> >& quatG1, std::vector<std::vector<double> >& poseG2, std::vector<std::vector<double> >& quatG2)
{   
    std::ofstream outf{file_name};
    std::vector<double> transErrorVector(n_poses, 0.0);
    std::vector<double> thetaErrorVector(n_poses, 0.0);
    
    for (unsigned int i=0; i<n_poses; ++i)
    {
        std::cout << "\n### Area " << i+1 << " ###\n";
        Eigen::Matrix4d E;
        if (poseG1.size() != n_poses)
        {
			std::vector<double> quat_empty, pose_empty;
			E = relocalization_error(poseO1[i], quatO1[i], poseO2[i], quatO2[i], pose_empty, quat_empty, pose_empty, quat_empty);
		} else 
		{
			E = relocalization_error(poseO1[i], quatO1[i], poseO2[i], quatO2[i], poseG1[i], quatG1[i], poseG2[i], quatG2[i]);
		}
        
        Eigen::Vector3d transE = E.block<3,1>(0,3);
        std::cout << "Translation error: X=" << transE[0] << " Y=" << transE[1] << " Z=" << transE[2];
        double transError = transE.norm();
        transErrorVector[i] = transError;
        std::cout << " Total=" << transError << std::endl;
        
        Eigen::Matrix3d rotE = E.block<3,3>(0,0);
        Eigen::AngleAxis<double> angleAxisE;
        angleAxisE.fromRotationMatrix(rotE);
        Eigen::Vector3d eulerE = rotE.eulerAngles(2,1,0); //yaw pitch roll 
        std::cout << "Rotation error: X=" << rad2deg(eulerE[2]) << " Y=" << rad2deg(eulerE[1]) << " Z=" << rad2deg(eulerE[0]);
        double thetaError = rad2deg(angleAxisE.angle()); //angle of the axis-angle representation as given by Rodrigue's formulas
        thetaErrorVector[i] = thetaError;
        std::cout << " Total=" << thetaError << std::endl;
        
        if (!file_name.empty())
        {                
            outf << "A" << i+1 << " & " << std::fixed << std::setprecision(6) << transError << " & " << thetaError << " \\\\" << std::endl;
        }
    }
    
    // --------  STATISTICS  --------
    // max
    double maxTrans = *std::max_element(transErrorVector.begin(), transErrorVector.end());
    double maxTheta = *std::max_element(thetaErrorVector.begin(), thetaErrorVector.end());
    
    // mean
    double meanTrans = 0.0;
    double meanTheta = 0.0;
    if (n_poses != 0) {
        meanTrans = accumulate(transErrorVector.begin(), transErrorVector.end(), 0.0) / n_poses;
        meanTheta = accumulate(thetaErrorVector.begin(), thetaErrorVector.end(), 0.0) / n_poses; 
    }
    
    // min
    double minTrans = *std::min_element(transErrorVector.begin(), transErrorVector.end());
    double minTheta = *std::min_element(thetaErrorVector.begin(), thetaErrorVector.end());
    
    std::cout << "\n----  STATISTICS  ----\n";
    std::cout << "Translation error\n";
    std::cout << "Max: " << maxTrans << std::endl;
    std::cout << "Mean: " << meanTrans << std::endl;
    std::cout << "Min: " << minTrans << std::endl;
    std::cout << "Rotation error\n";
    std::cout << "Max: " << maxTheta << std::endl;
    std::cout << "Mean: " << meanTheta << std::endl;
    std::cout << "Min: " << minTheta << std::endl;
    
    std::ofstream outf_statistics{file_name + "_statistics.txt"};
    if (!file_name.empty())
    {                
        outf_statistics << "max" << " & " << std::fixed << std::setprecision(6) << maxTrans << " & " << maxTheta << " \\\\" << std::endl;
        outf_statistics << "mean" << " & " << std::fixed << std::setprecision(6) << meanTrans << " & " << meanTheta << " \\\\" << std::endl;
        outf_statistics << "min" << " & " << std::fixed << std::setprecision(6) << minTrans << " & " << minTheta << " \\\\" << std::endl;
    }
}


void alignment_relocalization_error(const std::string& map_name, const std::string& traj_bag_name, const std::string& point_cloud_topic, const std::vector<double> timestamps, const bool bool_visualize, std::vector<std::vector<double> >& pose, std::vector<std::vector<double> >& quat, std::vector<std::vector<double> >& poseAligned, std::vector<std::vector<double> >& quatAligned)
{
    using pcl::GeneralizedIterativeClosestPoint;
    using pcl::PointCloud;
    using pcl::transformPointCloud;
    using namespace Eigen;
    
    pcl::PCDReader reader;
    
    PointCloud<PointT>::Ptr current_scan (new pcl::PointCloud<PointT>), map (new pcl::PointCloud<PointT>), transformed_scan (new pcl::PointCloud<PointT>);
    PointCloud<PointT>::Ptr filtered_scan (new pcl::PointCloud<PointT>), filtered_map (new pcl::PointCloud<PointT>), aligned_scan (new pcl::PointCloud<PointT>);
    
    reader.read(map_name, *map);
    rosbag::Bag bag;
    bag.open(traj_bag_name, rosbag::bagmode::Read);  
    
     for (unsigned int i=0;i<timestamps.size();i++){
        rosbag::View view(bag, rosbag::TopicQuery(point_cloud_topic), ros::Time(timestamps[i]-3.0), ros::Time(timestamps[i]+3.0));
        
        double min_timestamp_diff = ros::TIME_MAX.toSec();
        sensor_msgs::PointCloud2 min_scan;
        BOOST_FOREACH(rosbag::MessageInstance const m, view)
        {
            sensor_msgs::PointCloud2::ConstPtr scan_message = m.instantiate<sensor_msgs::PointCloud2>();
            double current_diff{std::abs(timestamps[i]-scan_message->header.stamp.toSec())};
            if (current_diff < min_timestamp_diff)
            {
                min_timestamp_diff = current_diff; 
                min_scan = sensor_msgs::PointCloud2{*scan_message};
            }
        }
        pcl::fromROSMsg(min_scan, *current_scan);  
              
        std::cout << "\n---- Alignment # " << i+1 << std::endl;
        std::cout << "current_scan has: " << current_scan->points.size () << " data points." << std::endl;
        std::cout << "map has: " << map->points.size () << " data points." << std::endl;
        
        // Create the filtering object
        pcl::VoxelGrid<PointT> sor;
        sor.setLeafSize (0.20f, 0.20f, 0.20f);    
        sor.setInputCloud (map);
        sor.filter (*filtered_map);
        sor.setLeafSize (0.10f, 0.10f, 0.10f); 
        sor.setInputCloud (current_scan);
        sor.filter (*filtered_scan);
        
        std::cout << "current_scan after filtering has: " << filtered_scan->points.size () << " data points." << std::endl;
        std::cout << "map after filtering has: " << filtered_map->points.size () << " data points." << std::endl;

        if (bool_visualize)
            visualize (map, filtered_scan);
        
        Vector3d eigen_pose{pose[i][0], pose[i][1], pose[i][2]};
        Quaterniond eigen_quat{quat[i][0], quat[i][1], quat[i][2], quat[i][3]};
        Matrix4d transform{Matrix4d::Identity()};
        transform.block<3,1>(0,3) = eigen_pose;
        transform.block<3,3>(0,0) = eigen_quat.toRotationMatrix();

        pcl::transformPointCloud (*filtered_scan, *transformed_scan, transform);
       
        if (bool_visualize)
            visualize (map, transformed_scan);
        
        // ICP-based alignment. Generalized ICP does (roughly) plane-to-plane
        // matching, and is much more robust than standard ICP.
        GeneralizedIterativeClosestPoint<PointT, PointT> icp;
        //pcl::IterativeClosestPoint<PointT, PointT> icp;  // STANDARD ICP  
        //stop criteria
        icp.setTransformationEpsilon(1e-8);
        icp.setEuclideanFitnessEpsilon (1);
        icp.setMaximumIterations(10);
        //other criteria
        icp.setMaxCorrespondenceDistance(0.30);
        //icp.setRANSACIterations(0);
        //icp.setMaximumOptimizerIterations(50); // default 20

        icp.setInputSource(transformed_scan);
        icp.setInputTarget(filtered_map);
        
        icp.align(*aligned_scan);
        
        if (bool_visualize)
            visualize (filtered_map, aligned_scan);    
        
        std::cout << "ICP alignment has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
        
        Eigen::Matrix4d icp_transform = icp.getFinalTransformation().cast<double>();
        Eigen::Matrix4d total_transform = transform * icp_transform;
        
        Vector3d total_trans{total_transform.block(0,3,3,1)};
        Matrix3d total_rotm{total_transform.block(0,0,3,3)};
        Quaterniond total_quat{total_rotm};
        
        //std::cout << total_transform << std::endl; 
        
        poseAligned[i] = std::vector<double>{total_trans(0), total_trans(1), total_trans(2)};
        quatAligned[i] = std::vector<double>{total_quat.w(), total_quat.x(), total_quat.y(), total_quat.z()};
    }
    
     bag.close();      
}

Eigen::Matrix4d relocalization_error(std::vector<double>& poseO1, std::vector<double>& quatO1, std::vector<double>& poseO2, std::vector<double>& quatO2, std::vector<double>& poseG1, std::vector<double>& quatG1, std::vector<double>& poseG2, std::vector<double>& quatG2)
{
    using namespace Eigen;
    
    Quaterniond eigen_quatO1{quatO1[0], quatO1[1], quatO1[2], quatO1[3]};
    Quaterniond eigen_quatO2{quatO2[0], quatO2[1], quatO2[2], quatO2[3]};
    Quaterniond eigen_quatG1{quatG1[0], quatG1[1], quatG1[2], quatG1[3]};
    Quaterniond eigen_quatG2{quatG2[0], quatG2[1], quatG2[2], quatG2[3]};
    
    Vector3d eigen_poseO1{poseO1[0], poseO1[1], poseO1[2]};
    Vector3d eigen_poseO2{poseO2[0], poseO2[1], poseO2[2]};
    Vector3d eigen_poseG1{poseG1[0], poseG1[1], poseG1[2]};
    Vector3d eigen_poseG2{poseG2[0], poseG2[1], poseG2[2]};
       
    // ### estimated poses ###
    Matrix4d TO1{Matrix4d::Identity()};
    TO1.block<3,1>(0,3) = eigen_poseO1;
    TO1.block<3,3>(0,0) = eigen_quatO1.toRotationMatrix();
    Matrix4d TO2{Matrix4d::Identity()};
    TO2.block<3,1>(0,3) = eigen_poseO2;
    TO2.block<3,3>(0,0) = eigen_quatO2.toRotationMatrix();
    Matrix4d PO = TO1.inverse()*TO2; // transformation matrix between estimated poses
    
    
    if (!poseG1.empty())
    {
		// ### ground truth ###
		Matrix4d TG1{Matrix4d::Identity()};
		TG1.block<3,1>(0,3) = eigen_poseG1;
		TG1.block<3,3>(0,0) = eigen_quatG1.toRotationMatrix();
		Matrix4d TG2{Matrix4d::Identity()};
		TG2.block<3,1>(0,3) = eigen_poseG2;
		TG2.block<3,3>(0,0) = eigen_quatG2.toRotationMatrix();
		Matrix4d PG = TG1.inverse()*TG2;
		
		// ### error ###
		Matrix4d E = PG.inverse()*PO;   
		
		return E;
	} else 
	{
		return PO;
	}  
}



void print_poses(std::vector<std::vector<double> >& poseO, std::vector<std::vector<double> >& quatO, std::vector<std::vector<double> >& poseG, std::vector<std::vector<double> >& quatG)
{
    std::cout << "----  POSES  ----" << std::endl;
    std::cout << "Odometry" << std::endl;
    for (std::vector<std::vector<double> >::size_type i=0; i < poseO.size(); i++)
    {
        std::cout << "poseO #" << i+1 << ": ";
        for (std::vector<double>::size_type j=0; j < poseO[i].size(); j++)
        {
            std::cout << poseO[i][j] << ' ';
        }
        std::cout << std::endl;
    }
    std::cout << "Ground truth" << std::endl;
    for (std::vector<std::vector<double> >::size_type i=0; i < poseG.size(); i++)
    {
        std::cout << "poseG #" << i+1 << ": ";
        for (std::vector<double>::size_type j=0; j < poseG[i].size(); j++)
        {
            std::cout << poseG[i][j] << ' ';
        }
        std::cout << std::endl;
    }
    
    std::cout << "----  QUATERNIONS  ----" << std::endl;
    std::cout << "Odometry" << std::endl;
    for (std::vector<std::vector<double> >::size_type i=0; i < quatO.size(); i++)
    {
        std::cout << "quatO #" << i+1 << ": ";
        for (std::vector<double>::size_type j=0; j < quatO[i].size(); j++)
        {
            std::cout << quatO[i][j] << ' ';
        }
        std::cout << std::endl;
    }
    std::cout << "Ground truth" << std::endl;
    for (std::vector<std::vector<double> >::size_type i=0; i < quatG.size(); i++)
    {
        std::cout << "quatG #" << i+1 << ": ";
        for (std::vector<double>::size_type j=0; j < quatG[i].size(); j++)
        {
            std::cout << quatG[i][j] << ' ';
        }
        std::cout << std::endl;
    }
}


void find_pose_quat(const std::string& traj_bag_name, const std::vector<double>& timestamps, const std::string& odometry_topic, const std::string& ground_truth_topic, std::vector<std::vector<double> >& poseO, std::vector<std::vector<double> >& quatO, std::vector<std::vector<double> >& poseG, std::vector<std::vector<double> >& quatG)
{
    rosbag::Bag bag;
    bag.open(traj_bag_name, rosbag::bagmode::Read);  
    std::vector<std::string> topics;
    bool no_ground_truth = true;
    topics.push_back(odometry_topic);
    if (!ground_truth_topic.empty())
    {
		topics.push_back(ground_truth_topic);
		no_ground_truth = false;
	}
    
    for (int i=0;i<timestamps.size();i++){
        bool found_odometry = false;
        bool found_ground_truth = false;
        rosbag::View view(bag, rosbag::TopicQuery(topics), ros::Time(timestamps[i]-0.5), ros::Time(timestamps[i]+0.5));
        
        double min_timestamp_diff = ros::TIME_MAX.toSec();
        std::vector<double> min_poseG(3, 0.0);
        std::vector<double> min_quatG(4, 0.0);
        BOOST_FOREACH(rosbag::MessageInstance const m, view)
        {
            if (m.getTopic() == odometry_topic)
            {
                nav_msgs::Odometry::ConstPtr odom_message = m.instantiate<nav_msgs::Odometry>();
                if (odom_message->header.stamp.toSec() == timestamps[i])
                {
                    poseO[i] = std::vector<double>{odom_message->pose.pose.position.x, odom_message->pose.pose.position.y, odom_message->pose.pose.position.z};
                    quatO[i] = std::vector<double>{odom_message->pose.pose.orientation.w, odom_message->pose.pose.orientation.x, odom_message->pose.pose.orientation.y, odom_message->pose.pose.orientation.z};
                    found_odometry = true;
                }
            }
            if (m.getTopic() == ground_truth_topic)
            {
				found_ground_truth = true;
                geometry_msgs::PoseStamped::ConstPtr ground_truth_message = m.instantiate<geometry_msgs::PoseStamped>();
                double current_diff{std::abs(timestamps[i]-ground_truth_message->header.stamp.toSec())};
                if (current_diff < min_timestamp_diff)
                {
                    min_timestamp_diff = current_diff; 
                    min_poseG = std::vector<double>{ground_truth_message->pose.position.x, ground_truth_message->pose.position.y, ground_truth_message->pose.position.z};
                    min_quatG = std::vector<double>{ground_truth_message->pose.orientation.w, ground_truth_message->pose.orientation.x, ground_truth_message->pose.orientation.y, ground_truth_message->pose.orientation.z};
                }
            }
        }
		if (found_ground_truth == false && no_ground_truth == false)
		{
			std::cerr << "A ground truth message has not been found in the current view"  << std::endl;
            exit(1);//exit or do additional error checking			  
		} else if (found_ground_truth == true && no_ground_truth == false)
		{
			poseG[i] = min_poseG;
			quatG[i] = min_quatG;	
		}
        
        if (found_odometry == false)
        {
            std::cerr << "Timestamp " << std::fixed << std::setprecision(9) << timestamps[i] << " has not been found among " + odometry_topic + " topic messages"  << std::endl;
            exit(1);//exit or do additional error checking
        }
    }
    
    bag.close();  
}

// ########## VISUALIZATION ########## 
void visualize (pcl::PointCloud<PointT>::Ptr source_cloud, pcl::PointCloud<PointT>::Ptr cloud_transformed)
{  
    // Visualization
    printf( "\nPoint cloud colors :  red  = original point cloud\n"
        "                        green  = transformed point cloud\n");
    pcl::visualization::PCLVisualizer viewer ("Point cloud rotation");
    
    // Define R,G,B colors for the point cloud
    pcl::visualization::PointCloudColorHandlerCustom<PointT> source_cloud_color_handler (source_cloud, 255, 0, 0);
    // We add the point cloud to the viewer and pass the color handler
    viewer.addPointCloud (source_cloud, source_cloud_color_handler, "original_cloud");

    pcl::visualization::PointCloudColorHandlerCustom<PointT> transformed_cloud_color_handler (cloud_transformed, 0, 255, 0); 
    viewer.addPointCloud (cloud_transformed, transformed_cloud_color_handler, "cloud_transformed");
    
    viewer.addCoordinateSystem (1.0, "cloud", 0);
    viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud_transformed");
    //viewer.setPosition(800, 400); // Setting visualiser window position

    while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
        viewer.spinOnce ();
    }
}

void writeToCSVfile(std::string name, Eigen::Matrix4f matrix)
{
    std::ofstream file(name.c_str());
    file << matrix.format(CSVFormat);
    file.close();
}

