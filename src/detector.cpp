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
#include <pcl/features/normal_3d.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <Eigen/Dense>
#include <math.h>
#define PI 3.14159265

using namespace Eigen;

ros::Publisher pub;
double threshold{.1};//.5  // set the height threshold
double deg_threshold{5};  // set the angle threshold in deg from the vertical

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

  try{
  	Tt2_v = tfBuffer.lookupTransform("scout_1_tf/base_footprint", (*cloud_msg).header.frame_id, ros::Time(0), ros::Duration(.5));
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

  	// define the vector of all normal parameters
  	typedef Matrix<float, 5, 1> Vector5f;
  	std::vector<Vector5f> plane_params;

  	std::vector<pcl::PointIndices> cluster_indices;
  	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  	ec.setSearchMethod(tree);
  	ec.setInputCloud(cloud);
  	ec.setClusterTolerance (.1);
  	ec.setMinClusterSize (50);
    //ec.setMaxClusterSize (1000);
  	ec.extract (cluster_indices); // Does the work

  	ROS_INFO("Number of clusters: %d", (int)cluster_indices.size());

  	int j = 0;
  	pcl::PointCloud<pcl::PointXYZ>::Ptr combined_cluster (new pcl::PointCloud<pcl::PointXYZ>);

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
      		//ROS_INFO("centroid: (%f, %f, %f)", centroid[0], centroid[1], centroid[2]);

      		//calculate nearest neighbours to the centroid to find the noraml to the plane

      		pcl::PointXYZ searchPoint(centroid[0],centroid[1],centroid[2]);  // input point -- set this as centroid
      		float radius =4; // radius of the neighbours
      		std::vector<int> pointIdxRadiusSearch; //to store index of surrounding points
      		std::vector<float> pointRadiusSquaredDistance; // to store distance to surrounding points

      		pcl::KdTreeFLANN<pcl::PointXYZ> tree;
      		tree.setInputCloud (cloud);

      		if ( tree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
      		{
          		// std::cout << "Looking for nearest neighbours " << std::endl;
          		// for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
          		//     std::cout << "    "  <<   cloud->points[ pointIdxRadiusSearch[i] ].x
          		//             << " " << cloud->points[ pointIdxRadiusSearch[i] ].y
          		//             << " " << cloud->points[ pointIdxRadiusSearch[i] ].z
          		//             << " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
      		}
          std::cout << "Cluster Size " << cloud_cluster->points.size () << std::endl;
      		//compute the normal to the point
      		pcl::NormalEstimation<pcl::PointXYZ,pcl::Normal>  normEst;
      		Eigen::Vector4f plane_parameters;
      		float curvature;
      		normEst.computePointNormal(*cloud, pointIdxRadiusSearch, plane_parameters, curvature);

      		//calculate angle of normal with respect to vertical :
      		double angle_rad, angle_deg;
      		double cos_angle;
      		cos_angle = plane_parameters[2]/std::pow(std::pow(plane_parameters[0],2) + std::pow(plane_parameters[1],2) + std::pow(plane_parameters[2],2), 0.5);
      		angle_rad = acos(cos_angle); // in radians
      		angle_deg = angle_rad*180/PI;

      		// Eigen::Vector5f param;
      		// param << plane_parameters, curvature;
      		// plane_params.push_back(param);
      		//find highest centroid and the corresponding group index
      		// if (centroid[2] >= highest){
      		//   highest_index = j;
      		//   highest = centroid[2];
      		// }
      		// // increment index of cluster
      		// j += 1;
      		if (centroid[2] > threshold && angle_deg > deg_threshold){
        		j = j+1;
        	//	std::cout << "Height of centroid -- " << centroid[2] << std::endl;
        	//	std::cout << "Inclination of normal --" << angle_deg << " deg " << std::endl;

        		for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end (); pit++){
            			combined_cluster->points.push_back(cloud->points[*pit]);
        		}
      		}
  	} // for all clusters
  	ROS_INFO("Number of clusters above threshold :(%d)", j);

  	// pcl::PointIndices highest_cluster = cluster_indices[highest_index];
  	// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster1 (new pcl::PointCloud<pcl::PointXYZ>);
  	// for (std::vector<int>::const_iterator pit = highest_cluster.indices.begin(); pit != highest_cluster.indices.end (); pit++){
  	//       cloud_cluster1->points.push_back(cloud->points[*pit]);
  	// }
  	// cloud_cluster1->width = int (cloud_cluster1->points.size ());
  	// cloud_cluster1->height = 1;
  	// cloud_cluster1->is_dense = true;

  	// Convert to ROS data type
	sensor_msgs::PointCloud2 output;
	pcl::toROSMsg(*combined_cluster, output);

	output.header= trns_cloud_msg.header;

	// Publish the data
	pub.publish (output);
  }// try
  catch (tf::TransformException ex){
  	ROS_ERROR("%s",ex.what());
  	ros::Duration(0.5).sleep();
  }
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/scout_1/inference/point_cloud", 4, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("/scout_1/inference/point_cloud_filtered", 1);

  // Spin
  ros::spin ();
}
