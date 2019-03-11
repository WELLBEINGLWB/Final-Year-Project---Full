// https://www.youtube.com/watch?v=s7kluSveBuE

// #include <librealsense2/rs.hpp>
#include "ros/ros.h"
#include "../include/base_realsense_node.h"
#include "../include/sr300_node.h"
#include "sensor_msgs/PointCloud2.h"
//#include "sensor_msgs/PointCloud2Modifier.h"
//#include "sensor_msgs/PointCloud2Iterator.h"
#include <stdlib.h>

int main(int argc, char **argv) {
     //Initializes ROS, and sets up a node
     ros::init(argc, argv, "point_cloud_publisher");
     ros::NodeHandle nh;

     //Ceates the publisher, and tells it to publish
     //to the husky/cmd_vel topic, with a queue size of 100
     //ros::Publisher pub=nh.advertise<geometry_msgs::Twist>("husky/cmd_vel", 100);
     ros::Publisher pub=nh.advertise<sensor_msgs::PointCloud2>("depth/color/points", 1);

     // The coordinate system is: X right, Y up, Z away from the camera. Units: Meters
     const float vertices[7][3] = {{1, 1, 1}, {1, 0, 0}, {1, 0, 1}, {2, 0 ,0}, {0, 1, 0}, {0, 0, 1}, {1, 1, 0}};
     bool use_texture = false; //(_pointcloud_texture.first != RS2_STREAM_ANY);
     unsigned char* color_data;
     int texture_width(0), texture_height(0);
     unsigned char no_color[3] = { 255, 255, 255 }; //WHITE
     if (use_texture)
     {

     }
     else
     {
         color_data = no_color;;
     }

     //const rs2::texture_coordinate* color_point = pc.get_texture_coordinates();
     int num_valid_points(0);
     // for (size_t point_idx=0; point_idx < pc.size(); point_idx++, color_point++)
     // {
     //     float i = static_cast<float>(color_point->u);
     //     float j = static_cast<float>(color_point->v);
     //
     //     if (i >= 0.f && i <= 1.f && j >= 0.f && j <= 1.f)
     //     {
     //         num_valid_points++;
     //     }
     // }



     //////////
     //Sets the loop to publish at a rate of 100Hz
     ros::Rate rate(100);


       while(ros::ok()) {
           // Define time
           ros::Time t;
           t = ros::Time::now();
           //Declares the message to be sent
           sensor_msgs::PointCloud2 msg_pointcloud;
           msg_pointcloud.header.stamp = t;
           msg_pointcloud.header.frame_id = "/camera_link"; //topic name to be published   ???
           msg_pointcloud.width = num_valid_points;
           msg_pointcloud.height = 1;
           msg_pointcloud.is_dense = true;

           sensor_msgs::PointCloud2Modifier modifier(msg_pointcloud);

           modifier.setPointCloud2Fields(4,
                                         "x", 1, sensor_msgs::PointField::FLOAT32,
                                         "y", 1, sensor_msgs::PointField::FLOAT32,
                                         "z", 1, sensor_msgs::PointField::FLOAT32,
                                         "rgb", 1, sensor_msgs::PointField::FLOAT32);
           modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");

           sensor_msgs::PointCloud2Iterator<float>iter_x(msg_pointcloud, "x");
           sensor_msgs::PointCloud2Iterator<float>iter_y(msg_pointcloud, "y");
           sensor_msgs::PointCloud2Iterator<float>iter_z(msg_pointcloud, "z");

           sensor_msgs::PointCloud2Iterator<uint8_t>iter_r(msg_pointcloud, "r");
           sensor_msgs::PointCloud2Iterator<uint8_t>iter_g(msg_pointcloud, "g");
           sensor_msgs::PointCloud2Iterator<uint8_t>iter_b(msg_pointcloud, "b");

           // Fill the PointCloud2 fields
           // const rs2::vertex* vertex = pc.get_vertices();

           // color_point = pc.get_texture_coordinates();

           // float color_pixel[2];
           // for (size_t point_idx=0; point_idx < pc.size(); vertex++, point_idx++, color_point++)
           // {
           //     float i(0), j(0);
           //     if (use_texture)
           //     {
           //         i = static_cast<float>(color_point->u);
           //         j = static_cast<float>(color_point->v);
           //     }
           //     if (i >= 0.f && i <= 1.f &&_pointcloud_publisher.publish(msg_pointcloud); j >= 0.f && j <= 1.f)
           //     {
           //         *iter_x = vertex->x;
           //         *iter_y = vertex->y;
           //         *iter_z = vertex->z;
           //
           //         color_pixel[0] = i * texture_width;
           //         color_pixel[1] = j * texture_height;
           //
           //         int pixx = static_cast<int>(color_pixel[0]);
           //         int pixy = static_cast<int>(color_pixel[1]);
           //         int offset = (pixy * texture_width + pixx) * 3;
           //         *iter_r = static_cast<uint8_t>(color_data[offset]);
           //         *iter_g = static_cast<uint8_t>(color_data[offset + 1]);
           //         *iter_b = static_cast<uint8_t>(color_data[offset + 2]);
           //
           //         ++iter_x; ++iter_y; ++iter_z;
           //         ++iter_r; ++iter_g; ++iter_b;
           //     }
           // }

           //Publish the message
           pub.publish(msg_pointcloud);
          //Delays untill it is time to send another message
          rate.sleep();
         }
}

//////////////////////////////////////////
// void myPublisher(rs2::points pc, const ros::Time& t, const rs2::frameset& frameset)
// {
//
//     // const float* vertices = readVertecesFromSocket();
//     // const float* texture = readTextureFromSocket();
//
//     bool use_texture = false; //(_pointcloud_texture.first != RS2_STREAM_ANY);
//     unsigned char* color_data;
//     int texture_width(0), texture_height(0);
//     unsigned char no_color[3] = { 255, 255, 255 }; //WHITE
//     if (use_texture)
//     {
//         // rs2::frame temp_frame = get_frame(frameset, _pointcloud_texture.first, _pointcloud_texture.second).as<rs2::video_frame>();
//         // if (!temp_frame.is<rs2::video_frame>())
//         // {
//         //     ROS_DEBUG_STREAM("texture frame not found");
//         //     return;
//         // }
//         //
//         // rs2::video_frame texture_frame = temp_frame.as<rs2::video_frame>();
//         // color_data = (uint8_t*)texture_frame.get_data();
//         // texture_width = texture_frame.get_width();
//         // texture_height = texture_frame.get_height();
//         // assert(texture_frame.get_bytes_per_pixel() == 3); // TODO: Need to support IR image texture.
//     }
//     else
//     {
//         color_data = no_color;;
//     }
//
//     const rs2::texture_coordinate* color_point = pc.get_texture_coordinates();
//     int num_valid_points(0);
//     for (size_t point_idx=0; point_idx < pc.size(); point_idx++, color_point++)
//     {
//         float i = static_cast<float>(color_point->u);
//         float j = static_cast<float>(color_point->v);
//
//         if (i >= 0.f && i <= 1.f && j >= 0.f && j <= 1.f)
//         {
//             num_valid_points++;
//         }
//     }
//
//     t = ros::Time::now();
//
//     sensor_msgs::PointCloud2 msg_pointcloud;
//     msg_pointcloud.header.stamp = t;
//     msg_pointcloud.header.frame_id = _optical_frame_id[DEPTH]; //topic name to be published   ???
//     msg_pointcloud.width = num_valid_points;
//     msg_pointcloud.height = 1;
//     msg_pointcloud.is_dense = true;
//
//     sensor_msgs::PointCloud2Modifier modi_pointcloud_publisherfier(msg_pointcloud);
//
//     modifier.setPointCloud2Fields(4,
//                                   "x", 1, sensor_msgs::PointField::FLOAT32,
//                                   "y", 1, sensor_msgs::PointField::FLOAT32,
//                                   "z", 1, sensor_msgs::PointField::FLOAT32,
//                                   "rgb", 1, sensor_msgs::PointField::FLOAT32);
//     modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
//
//     sensor_msgs::PointCloud2Iterator<float>iter_x(msg_pointcloud, "x");
//     sensor_msgs::PointCloud2Iterator<float>iter_y(msg_pointcloud, "y");
//     sensor_msgs::PointCloud2Iterator<float>iter_z(msg_pointcloud, "z");
//
//     sensor_msgs::PointCloud2Iterator<uint8_t>iter_r(msg_pointcloud, "r");
//     sensor_msgs::PointCloud2Iterator<uint8_t>iter_g(msg_pointcloud, "g");
//     sensor_msgs::PointCloud2Iterator<uint8_t>iter_b(msg_pointcloud, "b");
//
//     // Fill the PointCloud2 fields
//     const rs2::vertex* vertex = pc.get_vertices();
//     color_point = pc.get_texture_coordinates();
//
//     float color_pixel[2];
//     for (size_t point_idx=0; point_idx < pc.size(); vertex++, point_idx++, color_point++)
//     {
//         float i(0), j(0);
//         if (use_texture)
//         {
//             i = static_cast<float>(color_point->u);
//             j = static_cast<float>(color_point->v);
//         }
//         if (i >= 0.f && i <= 1.f &&_pointcloud_publisher.publish(msg_pointcloud); j >= 0.f && j <= 1.f)
//         {
//             *iter_x = vertex->x;
//             *iter_y = vertex->y;
//             *iter_z = vertex->z;
//
//             color_pixel[0] = i * texture_width;
//             color_pixel[1] = j * texture_height;
//
//             int pixx = static_cast<int>(color_pixel[0]);
//             int pixy = static_cast<int>(color_pixel[1]);
//             int offset = (pixy * texture_width + pixx) * 3;
//             *iter_r = static_cast<uint8_t>(color_data[offset]);
//             *iter_g = static_cast<uint8_t>(color_data[offset + 1]);
//             *iter_b = static_cast<uint8_t>(color_data[offset + 2]);
//
//             ++iter_x; ++iter_y; ++iter_z;
//             ++iter_r; ++iter_g; ++iter_b;
//         }
//     }
//     _pointcloud_publisher.publish(msg_pointcloud);
// }




///////////////////////
// int main (int argc, char **argv) {
//
//   ros::init(argc, argv, "turtle_lawnmower_node");
//
//
//
//   ros::spin();
//
//   return 0;
// }
