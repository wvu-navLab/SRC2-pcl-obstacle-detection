#include <ros/ros.h>
// PCL specific includes

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
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

ros::Publisher pub;


void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  double highest = 0.0; // finding the cluster with the highest index
  int highest_index = 0;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tf2_Listener(tfBuffer);
  geometry_msgs::TransformStamped transformStamped;
  geometry_msgs::TransformStamped Tt2_v;
  sensor_msgs::PointCloud2 trns_cloud_msg;

  // change frame of the point cloud

  Tt2_v = tfBuffer.lookupTransform("turtle2", "turtle1", ros::Time(0));
  tf2::doTransform(*cloud_msg, trns_cloud_msg, Tt2_v);

  // Convert from ROS to PCL data type
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg (trns_cloud_msg, *cloud);

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

  int j = 0;

  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it){ // iteract over all clusters, if necessary
  // if (cluster_indices.size() > 0){
  // 	std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); // Check if the first cluster is the bigger, or if we need to order the vector. Maybe check all the clusters

        // each cluster represented by it
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);

      for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end (); pit++){
          	cloud_cluster->points.push_back(cloud->points[*pit]);
      }
      cloud_cluster->width = int (cloud_cluster->points.size ());
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;

      // Find the center of the cluster (X, Y, Z), radius (parallel to x, y plane)

      Eigen::Vector4f centroid;
      pcl::compute3DCentroid (*cloud_cluster, centroid);
      ROS_INFO("centroid: (%f, %f, %f)", centroid[0], centroid[1], centroid[2]);

      //find highest centroid and the corresponding group index
      if (centroid[2] >= highest){
        highest_index = j;
        highest = centroid[2];
      }
      // increment index of cluster
      j += 1;
}

  pcl::PointIndices highest_cluster = cluster_indices[highest_index];
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster1 (new pcl::PointCloud<pcl::PointXYZ>);
  for (std::vector<int>::const_iterator pit = highest_cluster.indices.begin(); pit != highest_cluster.indices.end (); pit++){
        cloud_cluster1->points.push_back(cloud->points[*pit]);
  }
  cloud_cluster1->width = int (cloud_cluster1->points.size ());
  cloud_cluster1->height = 1;
  cloud_cluster1->is_dense = true;
	// Based on Z, decides if the cluster is an obstacle (point cloud is with respect to sensor. Need to transform to odom, for example)


  	// Convert to ROS data type
	sensor_msgs::PointCloud2 output;
	pcl::toROSMsg(*cloud_cluster1, output);

	output.header=cloud_msg->header;

	// Publish the data
	pub.publish (output);

}

int main (int argc, char** argv)
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
