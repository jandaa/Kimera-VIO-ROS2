#include <gflags/gflags.h>
#include <glog/logging.h>

// Nodes to be run in executable
#include "vio.h"

int main(int argc, char * argv[]){
  /**
   * Not sure if I need this
   **/
  // // Initialize Google's flags library.
  // google::ParseCommandLineFlags(&argc, &argv, true);

  // Initialize Google's logging library.
  // google::InitGoogleLogging(argv[0]);

  rclcpp::init(argc, argv);

  // Create node and initlize
  auto node = std::make_shared<KimeraVIO>();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}