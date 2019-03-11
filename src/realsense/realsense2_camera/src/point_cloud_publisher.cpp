#include "ros/ros.h"
#include "../include/base_realsense_node.h"
#include "sensor_msgs/PointCloud2.h"
#include <stdlib.h>


// void callback(msg) {
      // ros::Publisher pub=nh.advertise<sensor_msgs::PointCloud2>("/depth/points", 1);

      // sensor_msgs::PointCloud2 cloud_msg_1, cloud_msg_2;
      // cloud_msg_1.height = n_points;
      // cloud_msg_1.width = 1;
      // sensor_msgs::PointCloud2Modifier modifier(cloud_msg_1);
      // modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
      // cloud_msg_2 = cloud_msg_1;
      //
      // // Fill the point cloud
      // float point_data_raw[] = {0.6, 0.6, 0.6, 0.0, 0.0, 0.0, 0.5, 0.5, 0.5, 0.1, 0.1, 0.1, 0.3, 0.3, 0.3, 0.3, 0.9, 0.0, 0.31, 0.9, 0.0};

      // float point_data_raw[] = msg.points; ??????????????
      // int n_point = size_of(point_data_raw);
      // std::vector<float> point_data(point_data_raw, point_data_raw + 3*n_points);

        // uint8_t color_data_raw[n_point];
        // for(k = 0; k < n_point; k++) {
        //   color_data_raw[k] = 255;
        // }
      // uint8_t color_data_raw[] = {255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255};
      // std::vector<uint8_t> color_data(color_data_raw, color_data_raw + 3*n_points);
      //
      // float *data = reinterpret_cast<float*>(&cloud_msg_1.data.front());
      // for(size_t n=0, i=0; n<n_points; ++n) {
      //   for(; i<3*(n+1); ++i)
      //    *(data++) = point_data[i];
      //    // Add an extra float of padding
      //    ++data;
      //    uint8_t *bgr = reinterpret_cast<uint8_t*>(data++);
      //    // add the colors in order BGRA like PCL
      //    size_t j_max = 2;
      //    for(size_t j = 0; j <= j_max; ++j)
      //      *(bgr++) = color_data[3*n+(j_max - j)];
      //    // Add 3 extra floats of padding
      //    data += 3;
      //  }
      //
      //  // Fill using an iterator
      // sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg_2, "x");
      // sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(cloud_msg_2, "r");
      // sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(cloud_msg_2, "g");
      // sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(cloud_msg_2, "b");
      // for(size_t i=0; i<n_points; ++i, ++iter_x, ++iter_r, ++iter_g, ++iter_b) {
      //   for(size_t j=0; j<3; ++j)
      //     iter_x[j] = point_data[j+3*i];
      //   *iter_r = color_data[3*i];
      //   *iter_g = color_data[3*i+1];
      //   *iter_b = color_data[3*i+2];
      // }

      //       // Define time
      //       ros::Time t;
      //       t = ros::Time::now();
      //       // Declares the message to be sent
      //       sensor_msgs::PointCloud2 msg_pointcloud;
      //       cloud_msg_2.header.stamp = t;
      //       cloud_msg_2.header.frame_id = "camera_link";
      //       // cloud_msg_2.width = n_points;
      //       // cloud_msg_2.height = 1;
      //       cloud_msg_2.is_dense = true;
      //
      //       //Publish the message
      //       pub.publish(cloud_msg_2);


// }

int main(int argc, char **argv) {
     ros::init(argc, argv, "point_cloud_publisher");
     ros::NodeHandle nh;

     //Ceate publisher, publish to /depth/points topic, with a queue size of 100
     ros::Publisher pub=nh.advertise<sensor_msgs::PointCloud2>("/depth/points", 1);
     // ros::Subscriber sub = nh.subscribe ("/pointcloud??", 1, callback);

     int n_points = 7;

     sensor_msgs::PointCloud2 cloud_msg_1, cloud_msg_2;
     cloud_msg_1.height = n_points;
     cloud_msg_1.width = 1;
     sensor_msgs::PointCloud2Modifier modifier(cloud_msg_1);
     modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
     cloud_msg_2 = cloud_msg_1;

     // Fill the point cloud
     float point_data_raw[] = {0.6, 0.6, 0.6, 0.0, 0.0, 0.0, 0.5, 0.5, 0.5, 0.1, 0.1, 0.1, 0.3, 0.3, 0.3, 0.3, 0.9, 0.0, 0.31, 0.9, 0.0};
     std::vector<float> point_data(point_data_raw, point_data_raw + 3*n_points);
     uint8_t color_data_raw[] = {255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255};
     std::vector<uint8_t> color_data(color_data_raw, color_data_raw + 3*n_points);

     float *data = reinterpret_cast<float*>(&cloud_msg_1.data.front());
     for(size_t n=0, i=0; n<n_points; ++n) {
       for(; i<3*(n+1); ++i)
        *(data++) = point_data[i];
        // Add an extra float of padding
        ++data;
        uint8_t *bgr = reinterpret_cast<uint8_t*>(data++);
        // add the colors in order BGRA like PCL
        size_t j_max = 2;
        for(size_t j = 0; j <= j_max; ++j)
          *(bgr++) = color_data[3*n+(j_max - j)];
        // Add 3 extra floats of padding
        data += 3;
      }

      // Fill using an iterator
     sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg_2, "x");
     sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(cloud_msg_2, "r");
     sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(cloud_msg_2, "g");
     sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(cloud_msg_2, "b");
     for(size_t i=0; i<n_points; ++i, ++iter_x, ++iter_r, ++iter_g, ++iter_b) {
       for(size_t j=0; j<3; ++j)
         iter_x[j] = point_data[j+3*i];
       *iter_r = color_data[3*i];
       *iter_g = color_data[3*i+1];
       *iter_b = color_data[3*i+2];
     }

     //Publish at a rate of 100Hz
     ros::Rate rate(100);

       while(ros::ok()) {
           // Define time
           ros::Time t;
           t = ros::Time::now();
           // Declares the message to be sent
           sensor_msgs::PointCloud2 msg_pointcloud;
           cloud_msg_2.header.stamp = t;
           cloud_msg_2.header.frame_id = "camera_link";
           // cloud_msg_2.width = n_points;
           // cloud_msg_2.height = 1;
           cloud_msg_2.is_dense = true;

           //Publish the message
           pub.publish(cloud_msg_2);
          //Delay
          rate.sleep();
         }


         // while (ros::ok()){
         //   ros::spin();
         // }

}
