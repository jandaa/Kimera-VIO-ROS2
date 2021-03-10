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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)

find_package(OpenCV REQUIRED)
find_package(kimera_vio REQUIRED)
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

# build executable
add_executable(${PROJECT_NAME}_node 
  include/vio.h
  include/RosUtils.h
  src/vio_node.cpp
  src/vio.cpp
  src/RosUtils.cpp
)

ament_target_dependencies(
  ${PROJECT_NAME}_node 
    rclcpp 
    std_msgs
    sensor_msgs 
    geometry_msgs
    kimera_vio
    OpenCV
    Eigen3
    message_filters
)

target_link_libraries(
  ${PROJECT_NAME}_node 
    gtsam
)

target_include_directories(
  ${PROJECT_NAME}_node
    PRIVATE 
      /usr/local/include/gtsam/3rdparty/Eigen
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

install(
  DIRECTORY resources/
  DESTINATION resources
)

ament_package()
