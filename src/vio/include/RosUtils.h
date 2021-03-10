#pragma once

#include <string>

#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform.h>
#include <sensor_msgs/msg/camera_info.hpp>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <gtsam/geometry/Pose3.h>

#include <kimera-vio/frontend/CameraParams.h>
#include <kimera-vio/common/VioNavState.h>

namespace VIO {
namespace utils {

void msgTFtoPose(const geometry_msgs::msg::Transform& tf, gtsam::Pose3* pose);

void poseToMsgTF(const gtsam::Pose3& pose, geometry_msgs::msg::Transform* tf);

// void msgCamInfoToCameraParams(
//     const sensor_msgs::msg::CameraInfo::ConstSharedPtr& cam_info,
//     const std::string& base_link_frame_id,
//     const std::string& cam_frame_id,
//     CameraParams* cam_params
// );

void msgGtOdomToVioNavState(
    const nav_msgs::msg::Odometry& gt_odom,
    VioNavState* vio_navstate
);

}
}