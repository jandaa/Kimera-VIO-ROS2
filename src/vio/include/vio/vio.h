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

// Synchronization Policies
using CameraSyncPolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image, 
    sensor_msgs::msg::Image>;
using CameraInfoSyncPolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::CameraInfo, 
    sensor_msgs::msg::CameraInfo>;

// Synchronizers
using CameraSynchronizer = message_filters::Synchronizer<CameraSyncPolicy>;
using CameraInfoSynchronizer = message_filters::Synchronizer<CameraInfoSyncPolicy>;

class KimeraVIO : public rclcpp::Node
{
public:

    KimeraVIO();

private:

    void connect_vio();

    void camera_info_callback(
        const sensor_msgs::msg::CameraInfo::ConstSharedPtr left_msg,
        const sensor_msgs::msg::CameraInfo::ConstSharedPtr right_msg
    );

    void wait_for_camera_info();

private:

    // Subscriptions
    message_filters::Subscriber<sensor_msgs::msg::Image> left_image_sub;
    message_filters::Subscriber<sensor_msgs::msg::Image> right_image_sub;
    message_filters::Subscriber<sensor_msgs::msg::CameraInfo> left_camera_info_sub;
    message_filters::Subscriber<sensor_msgs::msg::CameraInfo> right_camera_info_sub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reint_flag_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr reint_pose_sub;

    // Message Synchronization
    std::unique_ptr<CameraSynchronizer> sync_cameras;
    std::unique_ptr<CameraInfoSynchronizer> sync_camera_params;

    // Pipeline
    VIO::Pipeline::UniquePtr vio_pipeline;
    VIO::RosDataProvider::Ptr data_provider;

    // Parameters
    VIO::VioParams::Ptr vio_params;
    std::string base_link_frame_id;
    std::string left_cam_frame_id;
    std::string right_cam_frame_id;

    // Flags
    bool camera_info_received = false;
};