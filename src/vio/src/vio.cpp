#include <cstdlib>
#include <memory>
#include <chrono>

#include <tf2_ros/static_transform_broadcaster.h>

#include <kimera-vio/pipeline/Pipeline-definitions.h>
#include <kimera-vio/pipeline/Pipeline.h>

#include "vio.h"
#include "RosUtils.h"

// Constant Parameters
static const rclcpp::Duration kMaxTimeSecsForCamInfo(10.0);

KimeraVIO::KimeraVIO(): 
Node("KimeraVIO")
{   
    // Initalize data parameters
    std::string params_folder_path;
    // this->vio_params = std::make_shared<VIO::VioParams>(params_folder_path);
    // this->data_provider = std::make_shared<VIO::RosDataProvider>(this, this->vio_params);

    // Create subscriptions (Takes topics from system environment variables)
    this->left_image_sub.subscribe(this, std::getenv("LEFT_IMAGE_TOPIC"));
    this->right_image_sub.subscribe(this, std::getenv("RIGHT_IMAGE_TOPIC"));
    this->left_camera_info_sub.subscribe(this, std::getenv("LEFT_CAMERA_INFO_TOPIC"));
    this->right_camera_info_sub.subscribe(this, std::getenv("RIGHT_CAMERA_INFO_TOPIC"));
    // imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
    //     std::getenv("IMU"),
    //     10,
    //     std::bind(&VIO::RosDataProvider::imu_callback, this->data_provider, std::placeholders::_1)
    // );
    // reint_flag_sub = this->create_subscription<std_msgs::msg::Bool>(
    //     "reinit_flag",
    //     10,
    //     std::bind(&KimeraVIO::reinit_callback, this, std::placeholders::_1)
    // );
    // reint_pose_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    //     "reinit_pose",
    //     10,
    //     std::bind(&KimeraVIO::reinit_pose_callaback, this, std::placeholders::_1)
    // );

    // Initialize Synchronizers
    this->sync_cameras = std::make_unique<CameraSynchronizer>(
        CameraSyncPolicy(10), 
        this->left_image_sub, 
        this->right_image_sub
    );
    this->sync_camera_params = std::make_unique<CameraInfoSynchronizer>(
        CameraInfoSyncPolicy(10), 
        this->left_camera_info_sub, 
        this->right_camera_info_sub
    );
    
    // Register callback functions
    // this->sync_cameras->registerCallback(
    //     std::bind(
    //         &VIO::RosDataProvider::camera_callback, 
    //         this->data_provider,
    //         std::placeholders::_1, std::placeholders::_2
    //     )
    // );
    this->sync_camera_params->registerCallback(
        std::bind(
            &KimeraVIO::camera_info_callback, 
            this,
            std::placeholders::_1, std::placeholders::_2
        )
    );

    this->base_link_frame_id = std::getenv("BASE_LINK_FRAME_ID");
    this->left_cam_frame_id = std::getenv("LEFT_CAM_FRAME_ID");
    this->right_cam_frame_id = std::getenv("RIGHT_CAM_FRAME_ID");

    // Wait for camera info to be received.
    this->wait_for_camera_info();
    this->connect_vio();
}

void KimeraVIO::connect_vio()
{
    // Register Data Provider callbacks
    this->data_provider->registerImuSingleCallback(
        std::bind(
            &VIO::Pipeline::fillSingleImuQueue,
            std::ref(*this->vio_pipeline),
            std::placeholders::_1
        )
    );

    this->data_provider->registerImuMultiCallback(
        std::bind(
            &VIO::Pipeline::fillMultiImuQueue,
            std::ref(*this->vio_pipeline),
            std::placeholders::_1
        )
    );

    std::bind(
        &VIO::RosDataProvider::camera_callback, 
        std::ref(this->data_provider),
        std::placeholders::_1, std::placeholders::_2
    );

    // this->data_provider->registerLeftFrameCallback(
    //     std::bind(
    //         &VIO::Pipeline::fillLeftFrameQueue,
    //         std::ref(*this->vio_pipeline),
    //         std::placeholders::_1
    //     )
    // );
    // VIO::utils::StatsCollectorImpl::AddSample

    // this->data_provider->registerRightFrameCallback(
    //     std::bind(
    //         &VIO::Pipeline::fillRightFrameQueue,
    //         std::ref(*this->vio_pipeline),
    //         std::placeholders::_1
    //     )
    // );
}

void KimeraVIO::wait_for_camera_info()
{
    auto start = this->now();
    auto current = this->now();
    while (!this->camera_info_received && (current - start) < kMaxTimeSecsForCamInfo) 
    {
        current = this->now();
    }
    RCLCPP_FATAL_STREAM_EXPRESSION(
        this->get_logger(),
        !this->camera_info_received,
        "Missing camera info, while trying for " << (current - start).seconds()
        << " seconds.\n"
        << "Expected camera info in topics:\n"
        << " - Left cam info topic: " << this->left_camera_info_sub.getTopic()
        << '\n'
        << " - Right cam info topic: " << this->right_camera_info_sub.getTopic();
    );
}

void KimeraVIO::camera_info_callback(
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr /*left_msg*/,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr /*right_msg*/
){
    // // Initialize CameraParams for pipeline.
    // VIO::utils::msgCamInfoToCameraParams(
    //     left_msg,
    //     this->base_link_frame_id,
    //     this->left_cam_frame_id,
    //     &this->vio_params->camera_params_.at(0)
    // );
    // VIO::utils::msgCamInfoToCameraParams(
    //     right_msg,
    //     this->base_link_frame_id,
    //     this->right_cam_frame_id,
    //     &this->vio_params->camera_params_.at(1)
    // );

    this->vio_params->camera_params_.at(0).print();
    this->vio_params->camera_params_.at(1).print();

    // Unregister this callback as it is no longer needed.
    RCLCPP_INFO(
        this->get_logger(), 
        "Unregistering CameraInfo subscribers as data has been received."
    );
    this->left_camera_info_sub.unsubscribe();
    this->right_camera_info_sub.unsubscribe();

    // Signal the correct reception of camera info
    this->camera_info_received = true;
}