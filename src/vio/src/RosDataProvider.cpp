#include <tf2_ros/static_transform_broadcaster.h>
#include <kimera-vio/pipeline/Pipeline.h>
#include <kimera-vio/frontend/CameraParams.h>

#include "vio/RosDataProvider.h"
#include "vio/RosUtils.h"

namespace VIO {

RosDataProvider::RosDataProvider() :
DataProviderInterface()
// nh(nh),
// vio_params(vio_params)
{
    nh = nullptr;
    vio_params = nullptr;
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

void RosDataProvider::reinit_callback()
{
    this->reinit_flag = true;
}

void RosDataProvider::reinit_pose_callaback(const geometry_msgs::msg::PoseStamped& reinit_pose)
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
        // CHECK_EQ(cv_ptr->encoding, sensor_msgs::image_encodings::MONO8)
        //     << "Expected image with MONO8, BGR8, or RGB8 encoding."
        //     "Add in here more conversions if you wish.";
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