#include <ros/ros.h>
// PCL specific includes
#include <string>

#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>
#include <tf2_ros/buffer.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <pcl/features/normal_3d.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <Eigen/Dense>
#include <math.h>
#include <pcl_obstacle_detection/HomingFilter.h>
#define PI 3.14159265

using namespace Eigen;

std::string odometry_child_frame_id;


bool serviceCallback(pcl_obstacle_detection::HomingFilter::Request &req, pcl_obstacle_detection::HomingFilter::Response &res){



  	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  	pcl::fromROSMsg (req.cloud, *cloud);

  	// This is necessary
  	std::vector<int> ind;
  	pcl::removeNaNFromPointCloud(*cloud, *cloud, ind);


  	// Perform the actual clustering
  	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  	tree->setInputCloud(cloud);


  	std::vector<pcl::PointIndices> cluster_indices;
  	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  	ec.setSearchMethod(tree);
  	ec.setInputCloud(cloud);
  	ec.setClusterTolerance (.1);
  	ec.setMinClusterSize (50);
    //ec.setMaxClusterSize (1000);
  	ec.extract (cluster_indices); // Does the work

  	ROS_WARN("PCL Homing Filter Number of clusters: %d", (int)cluster_indices.size());

  	int j = 0;
  	pcl::PointCloud<pcl::PointXYZ>::Ptr combined_cluster (new pcl::PointCloud<pcl::PointXYZ>);

  	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it){ // iteract over all clusters, if necessary

      		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);

      		for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end (); pit++){
          		cloud_cluster->points.push_back(cloud->points[*pit]);
      		}
      		cloud_cluster->width = int (cloud_cluster->points.size ());
      		cloud_cluster->height = 1;
      		cloud_cluster->is_dense = true;


        ROS_WARN_STREAM("Homing Cluster Size " << cloud_cluster->points.size ());
        		for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end (); pit++){
            			combined_cluster->points.push_back(cloud->points[*pit]);
        		}

  	} // for all clusters
	sensor_msgs::PointCloud2 output;
	pcl::toROSMsg(*combined_cluster, output);

	output.header= req.cloud.header;

	// Publish the data
	res.cloud = output;
  if((int)cluster_indices.size()==0){
    res.success = false;
  }
  else{
    res.success = true;
  }
  return true;
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "homing_filter_node");
  ros::NodeHandle nh;

  std::string node_name = "homing_filter_node";
  if(ros::param::get(node_name+"/odometry_child_frame_id",odometry_child_frame_id)==false)
  {
    ROS_FATAL("No parameter 'odometry_child_frame_id' specified");
    ros::shutdown();
    exit(1);
  }


  // Create a ROS service for the input point cloud
  ros::ServiceServer service = nh.advertiseService("homing_filter", serviceCallback);
  // Create a ROS publisher for the output point cloud

  // Spin
  ros::spin ();
}
