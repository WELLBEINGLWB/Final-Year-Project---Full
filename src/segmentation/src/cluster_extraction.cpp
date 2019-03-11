#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/visualization/cloud_viewer.h>
// #include <iostream>
// #include <pcl/io/io.h>
void viewerOneOff (pcl::visualization::PCLVisualizer& viewer)
{
    viewer.setBackgroundColor (1, 1, 1);
    // pcl::PointXYZ o;
    // o.x = 1.0;
    // o.y = 0;
    // o.z = 0;
    //viewer.addSphere (o, 0.25, "sphere", 0);
    std::cout << "I only run once" << std::endl;
}

// void viewerPsycho (pcl::visualization::PCLVisualizer& viewer)
// {
//     static unsigned count = 0;
//     std::stringstream ss;
//     ss << "Once per viewer loop: " << count++;
//     viewer.removeShape ("text", 0);
//     viewer.addText (ss.str(), 200, 300, "text", 0);
//
//     //FIXME: possible race condition here:
//     // user_data++;
// }

int
main (int argc, char** argv)
{
  // Read in the cloud data
  pcl::PCDReader reader;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
  reader.read ("/home/faisallab008/catkin_ws/src/segmentation/src/scene.pcd", *cloud);
  std::cout << "PointCloud before filtering has: " << cloud->points.size () << " data points." << std::endl; //*

  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  vg.setInputCloud (cloud);
  vg.setLeafSize (0.01f, 0.01f, 0.01f);
  vg.filter (*cloud_filtered);
  std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*

  // Create the segmentation object for the planar model and set all the parameters
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::PCDWriter writer;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.02);

  int i=0, nr_points = (int) cloud_filtered->points.size ();
  while (cloud_filtered->points.size () > 0.3 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);

    // Get the points associated with the planar surface
    extract.filter (*cloud_plane);
    std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_f);
    *cloud_filtered = *cloud_f;
  }

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud_filtered);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.02); // 2cm
  ec.setMinClusterSize (50); //original = 100
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_filtered);
  ec.extract (cluster_indices);

  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    // cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::io::loadPCDFile<pcl::PointXYZ> ("table_scene_lms400.pcd", *cloud);
    pcl::PointXYZ minPt, maxPt;
    pcl::getMinMax3D (*cloud_cluster, minPt, maxPt);
    std::cout << "Cluster " << j << ":" << std::endl;
    // get max and min xyz of each cluster
    // std::cout << "Max x: " << maxPt.x << std::endl;
    // std::cout << "Max y: " << maxPt.y << std::endl;
    // std::cout << "Max z: " << maxPt.z << std::endl;
    // std::cout << "Min x: " << minPt.x << std::endl;
    // std::cout << "Min y: " << minPt.y << std::endl;
    // std::cout << "Min z: " << minPt.z << std::endl;
    // Calculate dimensions of each cluster
    float xDim = maxPt.x - minPt.x;
    float yDim = maxPt.y - minPt.y;
    float zDim = maxPt.z - minPt.z;
    std::cout << "Dim x: " << xDim << std::endl;
    std::cout << "Dim y: " << yDim << std::endl;
    std::cout << "Dim z: " << zDim << std::endl;

    float xCenter = (maxPt.x + minPt.x)/2;
    float yCenter = (maxPt.y + minPt.y)/2;
    float zCenter = (maxPt.z + minPt.z)/2;
    std::cout << "Center x: " << xCenter << std::endl;
    std::cout << "Center y: " << yCenter << std::endl;
    std::cout << "Center z: " << zCenter << std::endl;

    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
    std::stringstream ss;
    ss << "cluster_" << j << ".pcd";
    writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
    j++;

  }

  ///////////// CLOUD VIEWER ////////////////////
  // char str[100];
  // std::cout << "Which file would you like to open?" << std::endl;
  // std::cin >> str;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud0 (new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::io::loadPCDFile ("cluster_0.pcd", *cloud0);
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::io::loadPCDFile ("cluster_1.pcd", *cloud1);
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::io::loadPCDFile ("cluster_2.pcd", *cloud2);
  pcl::visualization::CloudViewer viewer("Cloud Viewer");
  //blocks until the cloud is actually rendered
  viewer.showCloud(cloud0, "0");
  viewer.showCloud(cloud1, "1");
  viewer.showCloud(cloud2, "2");
  //This will only get called once
  viewer.runOnVisualizationThreadOnce (viewerOneOff);
  //This will get called once per visualization iteration
  // viewer.runOnVisualizationThread (viewerPsycho);
  while (!viewer.wasStopped ())
  {

  }

  return (0);
}
