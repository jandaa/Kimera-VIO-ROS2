#pragma once

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

// Kimera VIO
#include <kimera-vio/utils/Macros.h>
#include <kimera-vio/dataprovider/DataProviderInterface-definitions.h>
#include <kimera-vio/dataprovider/DataProviderInterface.h>
#include <kimera-vio/pipeline/Pipeline.h>

// OpenCV
#include <cv_bridge/cv_bridge.h>

namespace VIO
{

class RosDataProvider : public DataProviderInterface 
{
public:

    // KIMERA_DELETE_COPY_CONSTRUCTORS(RosDataProvider);
    KIMERA_POINTER_TYPEDEFS(RosDataProvider);

    RosDataProvider();

    virtual ~RosDataProvider();

    bool spin() override;

    void shutdown() override;

    void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg) const; 

    void camera_callback(
        const sensor_msgs::msg::Image::ConstSharedPtr left_msg,
        const sensor_msgs::msg::Image::ConstSharedPtr right_msg
    );

    void publish_static_transforms(
        const gtsam::Pose3& pose,
        const std::string& parent_frame_id,
        const std::string& child_frame_id
    );

    void reinit_callback(const std_msgs::msg::Bool::ConstSharedPtr reinit_flag);

    void reinit_pose_callaback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr reinit_pose);

    // Utility functions
    const cv::Mat readRosImage(const sensor_msgs::msg::Image::ConstSharedPtr& img_msg) const;

private:

    // Checks the current status of reinitialization flag
    inline bool getReinitFlag() const { return this->reinit_flag; }

    // Resets the current status of reinitialization flag
    inline void resetReinitFlag() { this->reinit_packet.resetReinitFlag(); }

private:

    // Parameters
    FrameId frame_count;

    // Upstream references
    rclcpp::Node* nh;
    VioParams::Ptr vio_params;  // Also a shared pointer

    // Misc
    bool reinit_flag = false;
    ReinitPacket reinit_packet = ReinitPacket();
};

}