#include <gflags/gflags.h>
#include <glog/logging.h>

// Nodes to be run in executable
#include "vio/vio.h"

int main(int argc, char * argv[])
{
  // Initialize Google's logging library.
  google::InitGoogleLogging("");

  rclcpp::init(argc, argv);

  // Create node and initialize
  auto node = std::make_shared<KimeraVIO>();
  // node->init();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}