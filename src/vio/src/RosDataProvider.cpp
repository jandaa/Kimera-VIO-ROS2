#include <tf2_ros/static_transform_broadcaster.h>
#include <kimera-vio/pipeline/Pipeline.h>
#include <kimera-vio/frontend/CameraParams.h>

#include "vio/RosDataProvider.h"
#include "vio/RosUtils.h"

// Constant Parameters
static const rclcpp::Duration kMaxTimeSecsForCamInfo(10.0);

namespace VIO {

RosDataProvider::RosDataProvider(rclcpp::Node* nh, VioParams::Ptr vio_params) :
DataProviderInterface(),
nh(nh)
{
    vio_params = vio_params;

    // Create subscriptions
    this->left_image_sub.subscribe(
        this->nh, 
        std::getenv("LEFT_IMAGE_TOPIC")
    );
    this->right_image_sub.subscribe(
        this->nh, 
        std::getenv("RIGHT_IMAGE_TOPIC")
    );
    this->imu_sub = this->nh->create_subscription<sensor_msgs::msg::Imu>(
        std::getenv("IMU_TOPIC"),
        10,
        std::bind(&RosDataProvider::imu_callback, this, std::placeholders::_1)
    );
    this->camera_info_sub = this->nh->create_subscription<sensor_msgs::msg::CameraInfo>(
        std::getenv("CAMERA_INFO_TOPIC"),
        10,
        std::bind(&RosDataProvider::camera_info_callback, this, std::placeholders::_1)
    );
    this->reint_flag_sub = this->nh->create_subscription<std_msgs::msg::Bool>(
        "reinit_flag",
        10,
        std::bind(&RosDataProvider::reinit_callback, this, std::placeholders::_1)
    );
    this->reint_pose_sub = this->nh->create_subscription<geometry_msgs::msg::PoseStamped>(
        "reinit_pose",
        10,
        std::bind(&RosDataProvider::reinit_pose_callaback, this, std::placeholders::_1)
    );

    // Initialize Synchronizers
    this->sync_cameras = std::make_unique<CameraSynchronizer>(
        CameraSyncPolicy(10), 
        this->left_image_sub, 
        this->right_image_sub
    );
    
    // Register callback functions
    this->sync_cameras->registerCallback(
        std::bind(
            &RosDataProvider::camera_callback, 
            this,
            std::placeholders::_1, std::placeholders::_2
        )
    );
    
    this->base_link_frame_id = std::getenv("BASE_FRAME_ID");
    this->left_cam_frame_id = std::getenv("LEFT_CAM_FRAME_ID");
    this->right_cam_frame_id = std::getenv("RIGHT_CAM_FRAME_ID");

    // Wait for camera info to be received.
    this->wait_for_camera_info();

}

RosDataProvider::~RosDataProvider() 
{
}

bool RosDataProvider::spin() 
{
    return false;
}

void RosDataProvider::shutdown() 
{
    return;
}

void RosDataProvider::wait_for_camera_info()
{
    auto start = this->nh->now();
    auto current = this->nh->now();
    while (!this->camera_info_received() && (current - start) < kMaxTimeSecsForCamInfo) 
    {
        current = this->nh->now();
    }

    RCLCPP_FATAL_STREAM_EXPRESSION(
        this->nh->get_logger(),
        !this->camera_info_received(),
        "Missing camera info, while trying for " << (current - start).seconds()
        << " seconds.\n"
        << "Expected camera info in topic: " << this->camera_info_sub->get_topic_name();
    );
}

void RosDataProvider::imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg) const
{
    ImuAccGyr imu_accgyr;

    imu_accgyr(0) = imu_msg->linear_acceleration.x;
    imu_accgyr(1) = imu_msg->linear_acceleration.y;
    imu_accgyr(2) = imu_msg->linear_acceleration.z;
    imu_accgyr(3) = imu_msg->angular_velocity.x;
    imu_accgyr(4) = imu_msg->angular_velocity.y;
    imu_accgyr(5) = imu_msg->angular_velocity.z;

    // Adapt imu timestamp to account for time shift in IMU-cam
    Timestamp timestamp = imu_msg->header.stamp.sec;

    imu_single_callback_(ImuMeasurement(timestamp, imu_accgyr));
}

void RosDataProvider::camera_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr left_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr right_msg
){

    // Make sure camera parameters exist for both cameras
    RCLCPP_ERROR_EXPRESSION(
        this->nh->get_logger(), 
        this->vio_params->camera_params_.size() != 2u,
        std::string("Missing camera parameters for one or both stereo cameras")
    );

    // Make sure camera parameters exist for both cameras
    RCLCPP_ERROR_EXPRESSION(
        this->nh->get_logger(), 
        !this->left_frame_callback_,
        std::string("Did you forget to register the left frame callback?")
    );

    RCLCPP_ERROR_EXPRESSION(
        this->nh->get_logger(), 
        !this->right_frame_callback_,
        std::string("Did you forget to register the right frame callback?")
    );

    const CameraParams& left_cam_info = this->vio_params->camera_params_.at(0);
    const CameraParams& right_cam_info = this->vio_params->camera_params_.at(1);

    const Timestamp& timestamp_left = left_msg->header.stamp.sec;
    const Timestamp& timestamp_right = right_msg->header.stamp.sec;

    this->left_frame_callback_(
        make_unique<Frame>(
            frame_count, 
            timestamp_left, 
            left_cam_info, 
            readRosImage(left_msg)
        )
    );
    this->right_frame_callback_(
        make_unique<Frame>(
            frame_count,
            timestamp_right,
            right_cam_info,
            readRosImage(right_msg)
        )
    );

    frame_count++;
}

void RosDataProvider::camera_info_callback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg)
{   
    // Find which camera the parameter is coming from
    unsigned int cam_ind;
    if (msg->header.frame_id == this->left_cam_frame_id)
    {   
        this->left_camera_info_received = true;
        cam_ind = 0;
    }
    if (msg->header.frame_id == this->right_cam_frame_id)
    {
        this->right_camera_info_received = true;
        cam_ind = 1;
    }
    else
    {
        return;
    }
    
    VIO::utils::msgCamInfoToCameraParams(
        msg,
        this->base_link_frame_id,
        msg->header.frame_id,
        &this->vio_params->camera_params_.at(cam_ind)
    );

    this->vio_params->camera_params_.at(cam_ind).print();

    if (this->camera_info_received())
    {
        // Unregister this callback as it is no longer needed.
        RCLCPP_INFO(
            this->nh->get_logger(), 
            "Unregistering CameraInfo subscribers as data has been received."
        );
        this->camera_info_sub.~__shared_ptr();
    }
}

void RosDataProvider::reinit_callback(const std_msgs::msg::Bool::ConstSharedPtr reinit_flag)
{
    this->reinit_flag = reinit_flag->data;
}

void RosDataProvider::reinit_pose_callaback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr reinit_pose)
{
    gtsam::Rot3 rotation(
        gtsam::Quaternion(
            reinit_pose->pose.orientation.w,
            reinit_pose->pose.orientation.x,
            reinit_pose->pose.orientation.y,
            reinit_pose->pose.orientation.z
        )
    );
    gtsam::Point3 position(
        reinit_pose->pose.position.x,
        reinit_pose->pose.position.y,
        reinit_pose->pose.position.z
    );

    this->reinit_packet.setReinitPose(gtsam::Pose3(rotation, position));
}

const cv::Mat RosDataProvider::readRosImage(
    const sensor_msgs::msg::Image::ConstSharedPtr& img_msg
) const
{
    // Convert image message to OpenCV image type
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvShare(img_msg);
    } catch (cv_bridge::Exception& exception) {
        RCLCPP_FATAL(
            this->nh->get_logger(),
            std::string("cv_bridge exception: ") + exception.what()
        );
        rclcpp::shutdown();
    }

    const cv::Mat img_const = cv_ptr->image;
    cv::Mat converted_img(img_const.size(), CV_8U);
    if (img_msg->encoding == sensor_msgs::image_encodings::BGR8) {
        RCLCPP_INFO(this->nh->get_logger(), "Converting image...");
        cv::cvtColor(img_const, converted_img, cv::COLOR_BGR2GRAY);
        return converted_img;
    } else if (img_msg->encoding == sensor_msgs::image_encodings::RGB8) {
        RCLCPP_INFO(this->nh->get_logger(), "Converting image...");
        cv::cvtColor(img_const, converted_img, cv::COLOR_RGB2GRAY);
        return converted_img;
    } else {
        return img_const;
    }
}

void RosDataProvider::publish_static_transforms(
    const gtsam::Pose3& pose,
    const std::string& parent_frame_id,
    const std::string& child_frame_id
) {
    static tf2_ros::StaticTransformBroadcaster static_broadcaster(this->nh);
    geometry_msgs::msg::TransformStamped static_transform_stamped;
    static_transform_stamped.header.stamp = this->nh->now();
    static_transform_stamped.header.frame_id = parent_frame_id;
    static_transform_stamped.child_frame_id = child_frame_id;
    VIO::utils::poseToMsgTF(pose, &static_transform_stamped.transform);
    static_broadcaster.sendTransform(static_transform_stamped);
}

} // namespace VIO