#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/visualization/cloud_viewer.h>

#include "pcl/common/eigen.h"
#include "pcl/common/transforms.h"
// #include <iostream>
// #include <pcl/io/io.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"

#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/Point.h"

#include "segmentation/seg.h"

ros::Publisher pub;
ros::Publisher pub2;
ros::Subscriber sub;
ros::Subscriber sub2;
// class sloudProcessor {
//   public:
//
// }

void gaze_callback(const geometry_msgs::Point::ConstPtr& msg) {

  float gaze_point[3] = {msg->x, msg->y, msg->z};

  std::cout << "X: " << gaze_point[0] << std::endl;
  std::cout << "Y: " << gaze_point[1] << std::endl;
  std::cout << "Z: " << gaze_point[2] << std::endl;
}

// void cloud_callback (const sensor_msgs::PointCloud2ConstPtr& cloud_msg){
void cloud_callback (const sensor_msgs::PointCloud2 &cloud_msg) {
     ROS_INFO("Callback");
     std_msgs::Float32MultiArray array;
     std_msgs::Float32MultiArray obj_interest;
     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pcl(new pcl::PointCloud<pcl::PointXYZ>);
     //Input of the above cloud and the corresponding output of cloud_pcl
     pcl::fromROSMsg(cloud_msg, *cloud_pcl);
     std::cout << "PointCloud: " << cloud_pcl->points.size () << " data points." << std::endl;

     pcl::PCDWriter writer;
     std::stringstream ss;
     ss << "main.pcd";
     writer.write<pcl::PointXYZ> (ss.str (), *cloud_pcl, false);

     // Passthrough filter to remove point beyond a distance treshold
     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_passthrough_filtered (new pcl::PointCloud<pcl::PointXYZ>);
     pcl::PassThrough<pcl::PointXYZ> pass;
     pass.setInputCloud(cloud_pcl);
     pass.setFilterFieldName ("z");
     pass.setFilterLimits (0.0f, 1.0f);
     //pass.setFilterLimitsNegative (true);
     pass.filter (*cloud_passthrough_filtered);
     writer.write<pcl::PointXYZ> ("filtered.pcd", *cloud_passthrough_filtered, false);

     // Transformations to rotate the point cloud
     float roty = M_PI/2.0;
     float rotz = -M_PI/2.0;
     Eigen::Affine3f transform_z = Eigen::Affine3f::Identity();
     Eigen::Affine3f transform_y = Eigen::Affine3f::Identity();
     transform_z.rotate (Eigen::AngleAxisf (rotz, Eigen::Vector3f::UnitZ()));
     transform_y.rotate (Eigen::AngleAxisf (roty, Eigen::Vector3f::UnitY()));
     pcl::PointCloud<pcl::PointXYZ>::Ptr transformed1_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
     pcl::PointCloud<pcl::PointXYZ>::Ptr transformed2_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
     pcl::transformPointCloud (*cloud_passthrough_filtered, *transformed1_cloud, transform_z);
     pcl::transformPointCloud (*transformed1_cloud, *transformed2_cloud, transform_y);


     // pcl::PCDReader reader;
     // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
     // reader.read ("/home/faisallab008/catkin_ws/src/segmentation/src/scene.pcd", *cloud);
     // std::cout << "PointCloud before filtering has: " << cloud->points.size () << " data points." << std::endl; //*

     // Create the filtering object: downsample the dataset using a leaf size of 1cm
     pcl::VoxelGrid<pcl::PointXYZ> vg;
     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
     vg.setInputCloud (transformed2_cloud);
     vg.setLeafSize (0.015f, 0.015f, 0.015f); // originally all 0.01f
     vg.filter (*cloud_filtered);
     std::cout << "Callback: PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*

     writer.write<pcl::PointXYZ> ("rotXZ.pcd", *cloud_filtered, false);

     // Create the segmentation object for the planar model and set all the parameters
     pcl::SACSegmentation<pcl::PointXYZ> seg;
     pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
     pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
     // pcl::PCDWriter writer;
     seg.setOptimizeCoefficients (true);
     seg.setModelType (pcl::SACMODEL_PLANE);
     seg.setMethodType (pcl::SAC_RANSAC);
     seg.setMaxIterations (100); //100
     seg.setDistanceThreshold (0.02); //0.02

     // std::stringstream ss;
     // ss << "main.pcd";
     // writer.write<pcl::PointXYZ> (ss.str (), *cloud_filtered, false);

     int i=0, nr_points = (int) cloud_filtered->points.size ();
     while (cloud_filtered->points.size () > 0.3 * nr_points)
     {
       // Segment the largest planar component from the remaining cloud
       seg.setInputCloud (cloud_filtered);
       seg.segment (*inliers, *coefficients);
       if (inliers->indices.size () == 0)
       {
         std::cout << "Callback: Could not estimate a planar model for the given dataset." << std::endl;
         break;
       }

       // Extract the planar inliers from the input cloud
       pcl::ExtractIndices<pcl::PointXYZ> extract;
       extract.setInputCloud (cloud_filtered);
       extract.setIndices (inliers);
       extract.setNegative (false);

       // Get the points associated with the planar surface
       extract.filter (*cloud_plane);
       std::cout << "Callback: PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

       // Remove the planar inliers, extract the rest
       extract.setNegative (true);
       extract.filter (*cloud_f);
       *cloud_filtered = *cloud_f;
     }

     // Calculate min and max of the removed table
     pcl::PointXYZ minTable, maxTable;
     pcl::getMinMax3D (*cloud_filtered, minTable, maxTable);
     std::cout << "Min table: " << minTable.z << std::endl;


     writer.write<pcl::PointXYZ> ("notable.pcd", *cloud_filtered, false);

     // Creating the KdTree object for the search method of the extraction
     pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
     tree->setInputCloud (cloud_filtered);

     std::vector<pcl::PointIndices> cluster_indices;
     pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
     ec.setClusterTolerance (0.02); // original = 2cm (0.02)
     ec.setMinClusterSize (20); //original = 100
     ec.setMaxClusterSize (25000);
     ec.setSearchMethod (tree);
     ec.setInputCloud (cloud_filtered);
     ec.extract (cluster_indices);

     // cloud to publish all clusters
     pcl::PointCloud<pcl::PointXYZ>::Ptr clustered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
     int j = 0;
     for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
     {
       pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
       pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_filtered (new pcl::PointCloud<pcl::PointXYZ>);
       for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
         cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
       cloud_cluster->width = cloud_cluster->points.size ();
       cloud_cluster->height = 1;
       cloud_cluster->is_dense = true;


      pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
      // build the filter
      outrem.setInputCloud(cloud_cluster);
      outrem.setRadiusSearch(0.05);
      outrem.setMinNeighborsInRadius (25); // 25 - 30 looks good
      // apply filter
      outrem.filter (*cluster_filtered);


       std::cout << "PointCloud representing the Cluster: " << cluster_filtered->points.size () << " data points." << std::endl;
       std::stringstream ss;
       ss << "CB_cluster_" << j << ".pcd";
       // writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*

       //////////////////////////////////////////////////////////////////////////
       //Find max and min xyz of each cluster
         pcl::PointXYZ minPt, maxPt;
         pcl::getMinMax3D (*cluster_filtered, minPt, maxPt);
         std::cout << "Cluster " << j << ":" << std::endl;

         // Get max and min xyz of each cluster
         // std::cout << "Max x: " << maxPt.x << std::endl;
         // std::cout << "Max y: " << maxPt.y << std::endl;
         // std::cout << "Max z: " << maxPt.z << std::endl;
         // std::cout << "Min x: " << minPt.x << std::endl;
         // std::cout << "Min y: " << minPt.y << std::endl;
         // std::cout << "Min z: " << minPt.z << std::endl;
         // minPt.z = minTable.z;
         // Calculate dimensions of each cluster
         float xDim = maxPt.x - minPt.x;
         float yDim = maxPt.y - minPt.y;
         float zDim = maxPt.z - minPt.z;
         // float zDim = maxPt.z - minTable.z;

         std::cout << "Dim x: " << xDim << std::endl;
         std::cout << "Dim y: " << yDim << std::endl;
         std::cout << "Dim z: " << zDim << std::endl;

         // // Calculate center point of each cluster
         // float xCenter = (maxPt.x + minPt.x)/2 - 0.30;
         // float yCenter = (maxPt.y + minPt.y)/2 - 0.18;
         // float zCenter = (maxPt.z + minPt.z)/2 + 0.08;
         float xCenter = (maxPt.x + minPt.x)/2;
         float yCenter = (maxPt.y + minPt.y)/2;
         float zCenter = (maxPt.z + minPt.z)/2;
         // float zCenter = (maxPt.z + minTable.z)/2;
         std::cout << "Center x: " << xCenter << std::endl;
         std::cout << "Center y: " << yCenter << std::endl;
         std::cout << "Center z: " << zCenter << std::endl;

         // Check if center point of cluster matches object of interest
         // float uncertainty = 0.42;
         // float gaze_point[3] = {1.2,0.5,0.2};
         // if(gaze_point[0] - uncertainty <= xCenter <= gaze_point[0] + uncertainty && gaze_point[1] - uncertainty <= yCenter <= gaze_point[1] + uncertainty && gaze_point[2] - uncertainty <= zCenter <= gaze_point[2] + uncertainty) {
         //   obj_interest.data.push_back(xDim);
         //   obj_interest.data.push_back(yDim);
         //   obj_interest.data.push_back(zDim);
         //   obj_interest.data.push_back(xCenter);
         //   obj_interest.data.push_back(yCenter);
         //   obj_interest.data.push_back(zCenter);
         // }
         // // float array[] = {xDim,yDim,zDim,xCenter,yCenter,zCenter};
         // else {
         //   array.data.push_back(xDim);
         //   array.data.push_back(yDim);
         //   array.data.push_back(zDim);
         //   array.data.push_back(xCenter);
         //   array.data.push_back(yCenter);
         //   array.data.push_back(zCenter);
         // }

         // float min_x = cluster_filtered->points[0].x, min_y = cluster_filtered->points[0].y, min_z = cluster_filtered->points[0].z, max_x = cluster_filtered->points[0].x, max_y = cluster_filtered->points[0].y, max_z = cluster_filtered->points[0].z;
         // // // pcl::StopWatch watch;
         // for (size_t i = 1; i < cluster_filtered->points.size (); ++i){
         //     if(cluster_filtered->points[i].x <= min_x )
         //         min_x = cluster_filtered->points[i].x;
         //     else if(cluster_filtered->points[i].y <= min_y )
         //         min_y = cluster_filtered->points[i].y;
         //     else if(cluster_filtered->points[i].z <= min_z )
         //         min_z = cluster_filtered->points[i].z;
         //     else if(cluster_filtered->points[i].x >= max_x )
         //         max_x = cluster_filtered->points[i].x;
         //     else if(cluster_filtered->points[i].y >= max_y )
         //         max_y = cluster_filtered->points[i].y;
         //     else if(cluster_filtered->points[i].z >= max_z )
         //         max_z = cluster_filtered->points[i].z;
         // }

         // std::cout << "New Max x: " << max_x << std::endl;
         // std::cout << "New Max y: " << max_y << std::endl;
         // std::cout << "New Max z: " << max_z << std::endl;
         // std::cout << "New Min x: " << min_x << std::endl;
         // std::cout << "New Min y: " << min_y << std::endl;
         // std::cout << "New Min z: " << min_z << std::endl;
         //
         // float xDim = max_x - min_x;
         // float yDim = max_y - min_y;
         // float zDim = max_z - min_z;
         // std::cout << "Dim x: " << xDim << std::endl;
         // std::cout << "Dim y: " << yDim << std::endl;
         // std::cout << "Dim z: " << zDim << std::endl;
         //
         // // Calculate center point of each cluster
         // float xCenter = (maxPt.x + minPt.x)/2 - 0.30;
         // float yCenter = (maxPt.y + minPt.y)/2 - 0.18;
         // float zCenter = (maxPt.z + minPt.z)/2 + 0.08;
         // float xCenter = (max_x + min_x)/2;
         // float yCenter = (max_y + min_y)/2;
         // float zCenter = (max_z + min_z)/2;
         // std::cout << "Center x: " << xCenter << std::endl;
         // std::cout << "Center y: " << yCenter << std::endl;
         // std::cout << "Center z: " << zCenter << std::endl;

         if(xCenter < 2.0) {
           array.data.push_back(xDim);
           array.data.push_back(yDim);
           array.data.push_back(zDim);
           array.data.push_back(xCenter);
           array.data.push_back(yCenter);
           array.data.push_back(zCenter);
         }
         // array.data.push_back(xDim);
         // array.data.push_back(yDim);
         // array.data.push_back(zDim);
         // array.data.push_back(xCenter);
         // array.data.push_back(yCenter);
         // array.data.push_back(zCenter);

         // Merge each cluster to one cloud
         *clustered_cloud += *cluster_filtered;
         j++;
     }
     pub.publish(array);
     writer.write<pcl::PointXYZ> ("clusters.pcd", *clustered_cloud, false);
     sensor_msgs::PointCloud2::Ptr clusters (new sensor_msgs::PointCloud2);
     pcl::toROSMsg (*clustered_cloud , *clusters);

	   clusters->header.frame_id = "/camera_link";
	   clusters->header.stamp=ros::Time::now();
     pub2.publish (*clusters);
     // clusters.data

     // pub.publish(obj_interest);
 		// ROS_INFO("Published!");
}

int
main (int argc, char** argv)
{

  ros::init(argc, argv, "segmentation");
  ros::NodeHandle nh;

  // sub = nh.subscribe ("/depth/points", 1, cloud_callback);
  std_msgs::Float32MultiArray array;

  sub = nh.subscribe ("/camera/depth_registered/points", 1, cloud_callback);
  sub2 = nh.subscribe ("/gaze", 1, gaze_callback);

  pub = nh.advertise<std_msgs::Float32MultiArray>("objects_array", 100);
  pub2= nh.advertise<sensor_msgs::PointCloud2>("cluster_cloud", 100);
  // std_msgs::Float32MultiArray array = cloud_callback;

  // std::cout << "TEST: PointCloud: " << &cloud_pcl->points.size () << " data points." << std::endl;

    while (ros::ok()){
      ros::spinOnce();
      sleep(5);
    }


  return (0);
}
