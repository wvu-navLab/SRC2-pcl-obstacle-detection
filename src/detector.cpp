#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

ros::Publisher pub;

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  
  // Convert from ROS to PCL data type
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);;
  pcl::fromROSMsg (*cloud_msg, *cloud);

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
  ec.setClusterTolerance (0.5);
  ec.setMinClusterSize (200);
  ec.extract (cluster_indices); // Does the work

  ROS_INFO("Number of clusters: %d", (int)cluster_indices.size());
  
  //for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it, j++) // iteract over all clusters, if necessary
  if (cluster_indices.size() > 0){
  	std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); // Check if the first cluster is the bigger, or if we need to order the vector. Maybe check all the clusters
  	
        // each cluster represented by it
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
  	for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end (); pit++){
        	cloud_cluster->points.push_back(cloud->points[*pit]);
  	}
  	cloud_cluster->width = int (cloud_cluster->points.size ());
  	cloud_cluster->height = 1;
  	cloud_cluster->is_dense = true;

	// Find the center of the cluster (X, Y, Z), radius (parallel to x, y plane)

	// Based on Z, decides if the cluster is an obstacle (point cloud is with respect to sensor. Need to transform to odom, for example)
  

  	// Convert to ROS data type
  	sensor_msgs::PointCloud2 output;
  	pcl::toROSMsg(*cloud_cluster, output);

  	output.header=cloud_msg->header;

  	// Publish the data
  	pub.publish (output);

  }

}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/scout_1/camera/points2", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

  // Spin
  ros::spin ();
}
