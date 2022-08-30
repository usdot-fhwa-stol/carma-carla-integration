#include "carla_sensors_integration_worker.h"
#include<ros/ros.h>

namespace carla_sensors
{
    

    void CarlaSensorsWorker::point_cloud_cb(sensor_msgs::PointCloud2 point_cloud)
    {

        if (point_cloud.data.size() == 0)
        {
             throw std::invalid_argument(" Invalid LIDAR Point Cloud Data");
            exit(0);
        }

        point_cloud_msg.header = point_cloud.header;
        point_cloud_msg.data = point_cloud.data;
        point_cloud_msg.fields = point_cloud.fields;
        point_cloud_msg.height = point_cloud.height;
        point_cloud_msg.is_bigendian = point_cloud.is_bigendian;
        point_cloud_msg.is_dense = point_cloud.is_dense;
        point_cloud_msg.point_step = point_cloud.point_step;
        point_cloud_msg.row_step = point_cloud.row_step;
        point_cloud_msg.width = point_cloud.width;


    }

    void CarlaSensorsWorker::image_raw_cb(sensor_msgs::Image image_raw)
    {

        if (image_raw.data.size() == 0)
        {
             throw std::invalid_argument("Invalid image data");
        }

        image_raw_msg.header = image_raw.header;
        image_raw_msg.data = image_raw.data;
        image_raw_msg.encoding = image_raw.encoding;
        image_raw_msg.height = image_raw.height;
        image_raw_msg.is_bigendian = image_raw.is_bigendian;
        image_raw_msg.step = image_raw.step;
        image_raw_msg.width = image_raw.width;


    }

    void CarlaSensorsWorker::image_color_cb(sensor_msgs::Image image_color)
    {

         if (image_color.data.size() == 0)
        {
             throw std::invalid_argument("Invalid image data");
        }

        image_color_msg.header = image_color.header;
        image_color_msg.data = image_color.data;
        image_color_msg.encoding = image_color.encoding;
        image_color_msg.height = image_color.height;
        image_color_msg.is_bigendian = image_color.is_bigendian;
        image_color_msg.step = image_color.step;
        image_color_msg.width = image_color.width;


    }

    void CarlaSensorsWorker::image_rect_cb(sensor_msgs::Image image_rect)
    {
        ROS_INFO_STREAM("CHECK_RECT");

         if (image_rect.data.size() == 0)
        {
             throw std::invalid_argument("Invalid image data");
        }

        image_rect_msg.header = image_rect.header;
        image_rect_msg.data = image_rect.data;
        image_rect_msg.encoding = image_rect.encoding;
        image_rect_msg.height = image_rect.height;
        image_rect_msg.is_bigendian = image_rect.is_bigendian;
        image_rect_msg.step = image_rect.step;
        image_rect_msg.width = image_rect.width;


    }

    void CarlaSensorsWorker::camera_info_cb(sensor_msgs::CameraInfo camera_info)
    {

        camera_info_msg.header = camera_info.header;
        camera_info_msg.height = camera_info.height;
        camera_info_msg.width = camera_info.width;
        camera_info_msg.distortion_model = camera_info.distortion_model;
        camera_info_msg.D = camera_info.D;
        camera_info_msg.K = camera_info.K;
        camera_info_msg.R = camera_info.R;
        camera_info_msg.P = camera_info.P;
        camera_info_msg.binning_x = camera_info.binning_x;
        camera_info_msg.binning_y = camera_info.binning_y;
        camera_info_msg.roi = camera_info.roi;



    }

    void CarlaSensorsWorker::gnss_fixed_fused_cb(sensor_msgs::NavSatFix gnss_fixed)
    {

        gnss_fixed_msg.header = gnss_fixed.header;
        gnss_fixed_msg.altitude = gnss_fixed.altitude;
        gnss_fixed_msg.latitude = gnss_fixed.latitude;
        gnss_fixed_msg.longitude = gnss_fixed.longitude;
        gnss_fixed_msg.position_covariance = gnss_fixed.position_covariance;
        gnss_fixed_msg.position_covariance_type = gnss_fixed.position_covariance_type;
        gnss_fixed_msg.status.status = gnss_fixed.status.status;

    
    }

    sensor_msgs::PointCloud2 CarlaSensorsWorker::get_lidar_msg()
    {
        return point_cloud_msg;
    }
    sensor_msgs::Image CarlaSensorsWorker::get_image_raw_msg()
    {
        return image_raw_msg;
    }
    sensor_msgs::Image CarlaSensorsWorker::get_image_color_msg()
    {
        return image_color_msg;
    }
    sensor_msgs::Image CarlaSensorsWorker::get_image_rect_msg()
    {
        return image_rect_msg;
    }

    sensor_msgs::CameraInfo CarlaSensorsWorker::get_camera_info()
    {
        return camera_info_msg;
    }


    gps_common::GPSFix CarlaSensorsWorker::get_gnss_fixed_msg()
    {
        return gnss_fixed_msg;
    }

}