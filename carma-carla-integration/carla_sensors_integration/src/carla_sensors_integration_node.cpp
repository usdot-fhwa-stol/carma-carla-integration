#include "carla_sensors_integration_node.h"
#include<ros/ros.h>

namespace carla_sensors
{
    void CarlaSensorsNode::initialize()
    {
        nh_.reset(new ros::CARMANodeHandle());
        points_raw_pub_ = nh_->advertise<sensor_msgs::PointCloud2>("points_raw", 1);
        image_raw_pub_= nh_->advertise<sensor_msgs::Image>("image_raw", 1);
        image_color_pub_ = nh_->advertise<sensor_msgs::Image>("image_color",1);
        image_rect_pub_ = nh_->advertise<sensor_msgs::Image>("image_rect", 1);
        camera_info_pub_ = nh_->advertise<sensor_msgs::CameraInfo>("camera_info", 1);
        gnss_fixed_fused_pub_ = nh_->advertise<gps_common::GPSFix>("gnss_fix_fused", 1);

        pnh_.reset(new ros::CARMANodeHandle());
        pnh_->getParam("role_name", carla_vehicle_role_);
        pnh_->getParam("object_detection_stream", object_detection_stream_enabled);
        pnh_->getParam("localization_stream", localization_stream_enabled);
        pnh_->getParam("carla_lidar_stream", carla_lidar_stream_enabled);
        pnh_->getParam("carla_camera_stream", carla_camera_stream_enabled);
        pnh_->getParam("carla_gnss_stream", carla_gnss_stream_enabled);

        //Subscribers

        /*Lidar*/
        if (!carla_lidar_stream_enabled)
            {
                ROS_ERROR_STREAM("CARLA LIDAR data stream is disabled");
            }
        else if (localization_stream_enabled && object_detection_stream_enabled)
            {
                throw std::invalid_argument("CARLA LIDAR sensor and both ground truth data streams cannot be enabled at the same time");
            }
        else
            {
                point_cloud_sub_ = nh_->subscribe<sensor_msgs::PointCloud2>("/carla/" + carla_vehicle_role_ +"/lidar/lidar/point_cloud", 10, &CarlaSensorsNode::point_cloud_cb, this);
            }

        /*Camera*/
        if (!carla_camera_stream_enabled)
            {
                ROS_ERROR_STREAM("CARLA camera data stream is disabled");
            }
        else if (carla_camera_stream_enabled && object_detection_stream_enabled)
            {
                throw std::invalid_argument("CARLA Camera sensor and ground truth object detection cannot be enabled at the same time");
            }
        else 
        {
            image_raw_sub_ = nh_->subscribe<sensor_msgs::Image>("/carla/" + carla_vehicle_role_ + "/camera/rgb/front/image", 10, &CarlaSensorsNode::image_raw_cb, this);
            image_color_sub_ = nh_->subscribe<sensor_msgs::Image>("/carla/" + carla_vehicle_role_ + "/camera/rgb/front/image_color", 10, &CarlaSensorsNode::image_color_cb, this);
            image_rect_sub_ = nh_->subscribe<sensor_msgs::Image>("/carla/" + carla_vehicle_role_ + "/camera/rgb/front/image_rect", 10, &CarlaSensorsNode::image_rect_cb, this);
        }
        /*GNSS*/
        if (!carla_gnss_stream_enabled)
            {
                ROS_ERROR_STREAM("CARLA camera data stream is disabled");
            }
        else if (carla_gnss_stream_enabled && localization_stream_enabled)
            {
                throw std::invalid_argument("CARLA GNSS sensor and ground truth localization cannot be enabled at the same time");
            }
        else
            {
                gnss_fixed_fused_sub_ = nh_->subscribe<sensor_msgs::NavSatFix>("/carla/" + carla_vehicle_role_ + "/gnss/novatel_gnss/fix", 10, &CarlaSensorsNode::gnss_fixed_fused_cb, this);
            }
        camera_info_sub_ = nh_->subscribe<sensor_msgs::CameraInfo>("/carla/" + carla_vehicle_role_ + "/camera/rgb/front/camera_info", 10, &CarlaSensorsNode::camera_info_cb, this);


    }

    void CarlaSensorsNode::run()
    {
        initialize();
        ros::CARMANodeHandle::spin();
    }

    void CarlaSensorsNode::point_cloud_cb(sensor_msgs::PointCloud2 point_cloud)
    {
        if (point_cloud.data.size() == 0)
        {
             throw std::invalid_argument(" Invalid LIDAR Point Cloud Data");
            exit(0);
        }
        if (point_cloud.header.stamp == point_cloud_msg.header.stamp && point_cloud_msg.header.seq != 0)
        {
            return;
        }

        carla_worker_.point_cloud_cb(point_cloud);

        point_cloud_msg = carla_worker_.get_lidar_msg();
        points_raw_pub_.publish(point_cloud_msg);

    }

    void CarlaSensorsNode::image_raw_cb(sensor_msgs::Image image_raw)
    {
        if (image_raw.data.size() == 0)
        {
             throw std::invalid_argument("Invalid image data");
        }
        carla_worker_.image_raw_cb(image_raw);
        image_raw_msg = carla_worker_.get_image_raw_msg();
        image_raw_pub_.publish(image_raw_msg);

    }

    void CarlaSensorsNode::image_color_cb(sensor_msgs::Image image_color)
    {

        if (image_color.data.size() == 0)
        {
             throw std::invalid_argument("Invalid image data");
        }

        carla_worker_.image_color_cb(image_color);
        image_color_msg = carla_worker_.get_image_color_msg();
        image_color_pub_.publish(image_color_msg);

    }

    void CarlaSensorsNode::image_rect_cb(sensor_msgs::Image image_rect)
    {
        if (image_rect.data.size() == 0)
        {
             throw std::invalid_argument("Invalid image data");
        }

        carla_worker_.image_rect_cb(image_rect);
        image_rect_msg = carla_worker_.get_image_rect_msg();
        image_rect_pub_.publish(image_rect_msg);

    }

    void CarlaSensorsNode::camera_info_cb(sensor_msgs::CameraInfo camera_info)
    {

        carla_worker_.camera_info_cb(camera_info);

        camera_info_msg = carla_worker_.get_camera_info(); 
        camera_info_pub_.publish(camera_info_msg);

    }

    void CarlaSensorsNode::gnss_fixed_fused_cb(sensor_msgs::NavSatFix gnss_fixed)
    {
        

        carla_worker_.gnss_fixed_fused_cb(gnss_fixed);
        gnss_fixed_msg = carla_worker_.get_gnss_fixed_msg();
        gnss_fixed_fused_pub_.publish(gnss_fixed_msg);

    
    }
}