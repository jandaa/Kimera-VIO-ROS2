#pragma once

// STD
#include <memory>

// ROS
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.h>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

// Message Filtering
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>

// Kimera VIO
#include <kimera-vio/utils/Macros.h>
#include <kimera-vio/pipeline/Pipeline-definitions.h>
#include <kimera-vio/pipeline/Pipeline.h>
#include <kimera-vio/dataprovider/DataProviderInterface-definitions.h>
#include <kimera-vio/dataprovider/DataProviderInterface.h>

// OpenCV
#include "cv_bridge/cv_bridge.h"

#include "RosDataProvider.h"

class KimeraVIO : public rclcpp::Node
{
public:

    KimeraVIO();

    void init();

private:

    void connect_vio();

    void declare_parameters();

private:

    // Pipeline
    VIO::Pipeline::UniquePtr vio_pipeline;
    VIO::RosDataProvider::Ptr data_provider;

    // Parameters
    VIO::VioParams::Ptr vio_params;
};