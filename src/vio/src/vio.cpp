#include <cstdlib>
#include <memory>
#include <chrono>

#include <tf2_ros/static_transform_broadcaster.h>

#include <kimera-vio/pipeline/Pipeline-definitions.h>
#include <kimera-vio/pipeline/Pipeline.h>

#include "vio/vio.h"
#include "vio/RosUtils.h"

KimeraVIO::KimeraVIO(): 
Node("KimeraVIO")
{   
    // Initalize data parameters
    std::string params_folder_path;
    this->vio_params = std::make_shared<VIO::VioParams>(params_folder_path);
    this->data_provider = std::make_shared<VIO::RosDataProvider>(
        reinterpret_cast<rclcpp::Node*>(this), 
        this->vio_params
    );

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

    this->data_provider->registerLeftFrameCallback(
        std::bind(
            &VIO::Pipeline::fillLeftFrameQueue,
            std::ref(*this->vio_pipeline),
            std::placeholders::_1
        )
    );

    this->data_provider->registerRightFrameCallback(
        std::bind(
            &VIO::Pipeline::fillRightFrameQueue,
            std::ref(*this->vio_pipeline),
            std::placeholders::_1
        )
    );
}