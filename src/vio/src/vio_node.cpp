#include <gflags/gflags.h>
#include <glog/logging.h>

// Nodes to be run in executable
#include "vio/vio.h"

int main(int argc, char * argv[]){

  // Initialize Google's flags library.
  google::ParseCommandLineFlags(&argc, &argv, true);

  // Initialize Google's logging library.
  google::InitGoogleLogging(argv[0]);

  rclcpp::init(argc, argv);

  // Create node and initialize
  // auto vio_params = std::make_shared<VIO::VioParams>("test");
  auto node = std::make_shared<KimeraVIO>();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}