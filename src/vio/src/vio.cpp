#include <cstdlib>
#include <memory>
#include <chrono>

#include <tf2_ros/static_transform_broadcaster.h>

#include "vio.h"
#include "RosUtils.h"

using namespace std::chrono_literals;

// Constant Parameters
static constexpr size_t kMaxCamInfoQueueSize = 10u;
static constexpr size_t kMaxCamInfoSynchronizerQueueSize = 10u;
static const rclcpp::Duration kMaxTimeSecsForCamInfo(10.0);

KimeraVIO::KimeraVIO(const VIO::VioParams& vio_params): 
Node("KimeraVIO"),
DataProviderInterface(),
vio_params(vio_params)
{   
    // Create subscriptions (Takes topics from system environment variables)
    this->imu_sub.subscribe(this, std::getenv("IMU"));
    this->left_image_sub.subscribe(this, std::getenv("LEFT_IMAGE_TOPIC"));
    this->right_image_sub.subscribe(this, std::getenv("RIGHT_IMAGE_TOPIC"));
    this->left_camera_info_sub.subscribe(this, std::getenv("LEFT_CAMERA_INFO_TOPIC"));
    this->right_camera_info_sub.subscribe(this, std::getenv("RIGHT_CAMERA_INFO_TOPIC"));
    this->reint_flag_sub = this->create_subscription<std_msgs::msg::Bool>(
        "reinit_flag",
        10,
        std::bind(&KimeraVIO::reinit_callback, this, std::placeholders::_1)
    );
    this->reint_pose_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "reinit_pose",
        10,
        std::bind(&KimeraVIO::reinit_pose_callaback, this, std::placeholders::_1)
    );

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
    this->sync_cameras->registerCallback(
        std::bind(
            &KimeraVIO::camera_callback, 
            this,
            std::placeholders::_1, std::placeholders::_2
        )
    );
    this->sync_camera_params->registerCallback(
        std::bind(
            &KimeraVIO::camera_info_callback, 
            this,
            std::placeholders::_1, std::placeholders::_2
        )
    );

    // Wait for camera info to be received.
    this->wait_for_camera_info();

    this->base_link_frame_id = std::getenv("BASE_LINK_FRAME_ID");
    this->left_cam_frame_id = std::getenv("LEFT_CAM_FRAME_ID");
    this->right_cam_frame_id = std::getenv("RIGHT_CAM_FRAME_ID");
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
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr left_msg,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr right_msg
){
    // Initialize CameraParams for pipeline.
    VIO::utils::msgCamInfoToCameraParams(
        left_msg,
        this->base_link_frame_id,
        this->left_cam_frame_id,
        &this->vio_params.camera_params_.at(0)
    );
    VIO::utils::msgCamInfoToCameraParams(
        right_msg,
        this->base_link_frame_id,
        this->right_cam_frame_id,
        &this->vio_params.camera_params_.at(1)
    );

    this->vio_params.camera_params_.at(0).print();
    this->vio_params.camera_params_.at(1).print();

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

void KimeraVIO::camera_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr left_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr right_msg
){
    RCLCPP_INFO_STREAM(
        this->get_logger(), 
        "Receiving synched camera measurements with timestames: " << std::endl
        <<  "\t Left Cam: " << left_msg->header.stamp.sec << ":" << left_msg->header.stamp.nanosec
        <<  "\t right Cam: " << right_msg->header.stamp.sec << ":" << right_msg->header.stamp.nanosec
    );

    // Make sure camera parameters exist for both cameras
    RCLCPP_ERROR_EXPRESSION(
        this->get_logger(), 
        vio_params.camera_params_.size() != 2u,
        std::string("Missing camera parameters for one or both stereo cameras")
    );

    // Make sure camera parameters exist for both cameras
    RCLCPP_ERROR_EXPRESSION(
        this->get_logger(), 
        !left_frame_callback_,
        std::string("Did you forget to register the left frame callback?")
    );

    RCLCPP_ERROR_EXPRESSION(
        this->get_logger(), 
        !right_frame_callback_,
        std::string("Did you forget to register the right frame callback?")
    );

    const VIO::CameraParams& left_cam_info = vio_params.camera_params_.at(0);
    const VIO::CameraParams& right_cam_info = vio_params.camera_params_.at(1);

    const VIO::Timestamp& timestamp_left = left_msg->header.stamp.sec;
    const VIO::Timestamp& timestamp_right = right_msg->header.stamp.sec;

    left_frame_callback_(
        VIO::make_unique<VIO::Frame>(
            frame_count, timestamp_left, left_cam_info, readRosImage(left_msg)
        )
    );
    right_frame_callback_(
        VIO::make_unique<VIO::Frame>(
            frame_count,
            timestamp_right,
            right_cam_info,
            readRosImage(right_msg)
        )
    );
    frame_count++;

}

void KimeraVIO::imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg) const
{
    RCLCPP_INFO_STREAM(
        this->get_logger(), 
        "Recieving IMU measurements with timestame: "   << imu_msg->header.stamp.sec << ":" 
                                                        << imu_msg->header.stamp.nanosec
    );

    VIO::ImuAccGyr imu_accgyr;

    imu_accgyr(0) = imu_msg->linear_acceleration.x;
    imu_accgyr(1) = imu_msg->linear_acceleration.y;
    imu_accgyr(2) = imu_msg->linear_acceleration.z;
    imu_accgyr(3) = imu_msg->angular_velocity.x;
    imu_accgyr(4) = imu_msg->angular_velocity.y;
    imu_accgyr(5) = imu_msg->angular_velocity.z;

    // Adapt imu timestamp to account for time shift in IMU-cam
    VIO::Timestamp timestamp = imu_msg->header.stamp.sec;

    imu_single_callback_(VIO::ImuMeasurement(timestamp, imu_accgyr));
}

void KimeraVIO::reinit_callback(const std_msgs::msg::Bool::ConstPtr& reinit_flag)
{
    this->reinit_flag = true;
}

void KimeraVIO::reinit_pose_callaback(const geometry_msgs::msg::PoseStamped& reinit_pose)
{
    gtsam::Rot3 rotation(
        gtsam::Quaternion(
            reinit_pose.pose.orientation.w,
            reinit_pose.pose.orientation.x,
            reinit_pose.pose.orientation.y,
            reinit_pose.pose.orientation.z
        )
    );
    gtsam::Point3 position(
        reinit_pose.pose.position.x,
        reinit_pose.pose.position.y,
        reinit_pose.pose.position.z
    );

    this->reinit_packet.setReinitPose(gtsam::Pose3(rotation, position));
}

void KimeraVIO::publish_static_transforms(
    const gtsam::Pose3& pose,
    const std::string& parent_frame_id,
    const std::string& child_frame_id
) {
    static tf2_ros::StaticTransformBroadcaster static_broadcaster(this);
    geometry_msgs::msg::TransformStamped static_transform_stamped;
    static_transform_stamped.header.stamp = this->now();
    static_transform_stamped.header.frame_id = parent_frame_id;
    static_transform_stamped.child_frame_id = child_frame_id;
    VIO::utils::poseToMsgTF(pose, &static_transform_stamped.transform);
    static_broadcaster.sendTransform(static_transform_stamped);
}

const cv::Mat KimeraVIO::readRosImage(
    const sensor_msgs::msg::Image::ConstSharedPtr& img_msg
) const
{
    // Convert image message to OpenCV image type
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvShare(img_msg);
    } catch (cv_bridge::Exception& exception) {
        RCLCPP_FATAL(
            this->get_logger(),
            std::string("cv_bridge exception: ") + exception.what()
        );
        rclcpp::shutdown();
    }

    const cv::Mat img_const = cv_ptr->image;
    cv::Mat converted_img(img_const.size(), CV_8U);
    if (img_msg->encoding == sensor_msgs::image_encodings::BGR8) {
        RCLCPP_INFO(this->get_logger(), "Converting image...");
        cv::cvtColor(img_const, converted_img, cv::COLOR_BGR2GRAY);
        return converted_img;
    } else if (img_msg->encoding == sensor_msgs::image_encodings::RGB8) {
        RCLCPP_INFO(this->get_logger(), "Converting image...");
        cv::cvtColor(img_const, converted_img, cv::COLOR_RGB2GRAY);
        return converted_img;
    } else {
        CHECK_EQ(cv_ptr->encoding, sensor_msgs::image_encodings::MONO8)
            << "Expected image with MONO8, BGR8, or RGB8 encoding."
            "Add in here more conversions if you wish.";
        return img_const;
    }
}