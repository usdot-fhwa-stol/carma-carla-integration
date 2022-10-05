/*
 * Copyright (C) 2022 LEIDOS.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

#pragma once

#include <ros/ros.h>
#include <carma_utils/CARMANodeHandle.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/NavSatFix.h>
#include <gps_common/GPSFix.h>

namespace ds_toggle
{
    class DataStreamToggle
    {
        public:
            void initialize();
            void run();


        private:
            std::shared_ptr<ros::NodeHandle> pnh_;
            bool vehicle_perception_stream_enabled;
            bool vehicle_position_stream_enabled;
            bool carla_lidar_stream_enabled;
            bool carla_camera_stream_enabled;
            bool carla_gnss_stream_enabled;


    };
}
