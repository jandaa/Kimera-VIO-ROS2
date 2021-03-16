cmake_minimum_required(VERSION 3.5)
project(vio)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(gflags_DIR "/usr/lib/x86_64-linux-gnu/cmake/gflags")
# set(cv_bridge_DIR "/ws/vision_opencv/install/cv_bridge/share/cv_bridge/cmake")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(gflags REQUIRED)
find_library(glog HINTS "/usr/lib/x86_64-linux-gnu")
find_library(kimera_vio HINTS "/usr/local/lib")

find_package(OpenCV 3.3.1 REQUIRED)
find_package(cv_bridge REQUIRED)
# find_package(kimera_vio REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(
  Boost REQUIRED 
  COMPONENTS
    serialization 
    system 
    filesystem 
    thread 
    date_time
    regex 
    timer 
    chrono
)
if(NOT TARGET Boost::boost)
  add_library(Boost::boost INTERFACE IMPORTED)
  set_target_properties(Boost::boost PROPERTIES
  INTERFACE_LINK_LIBRARIES "${Boost_LIBRARIES}"
  INTERFACE_INCLUDE_DIRECTORIES "${Boost_INCLUDE_DIRS}")
endif()

# build executable
add_executable(${PROJECT_NAME}_node 
  include/vio/vio.h
  src/vio.cpp
  include/vio/RosUtils.h
  src/RosUtils.cpp
  include/vio/RosDataProvider.h
  src/RosDataProvider.cpp
  src/vio_node.cpp
)

ament_target_dependencies(
  ${PROJECT_NAME}_node 
    rclcpp 
    std_msgs
    sensor_msgs 
    geometry_msgs
    OpenCV
    cv_bridge
    tf2_ros
    Eigen3
    message_filters
)

target_link_libraries(${PROJECT_NAME}_node 
  gtsam
  gflags
  glog
  kimera_vio
)

target_include_directories(
  ${PROJECT_NAME}_node
    PRIVATE 
      /usr/local/include/gtsam/3rdparty/Eigen
      /usr/include/glog
      /usr/local/include/kimera-vio
    PUBLIC
      include/
)

install(
  TARGETS
    ${PROJECT_NAME}_node
  DESTINATION 
    lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

# install(
#   DIRECTORY resources/
#   DESTINATION resources
# )

# add_compile_options(-g)

ament_package()
