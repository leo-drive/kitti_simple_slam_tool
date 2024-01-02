cmake_minimum_required(VERSION 3.5)
project(kitti_simple_slam_tool)

#Default to C++14
if(NOT CMAKE_CXX_STANDARD)
set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
add_compile_options(-Wall -Wextra -Wpedantic)
endif()
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
#find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})

include_directories(${PCL_INCLUDE_DIRS})
include_directories(${pcl_conversions_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(${PROJECT_NAME}  src/kitti_simple_slam_tool.cpp)
ament_target_dependencies(${PROJECT_NAME} Eigen3 rclcpp std_msgs sensor_msgs nav_msgs message_filters pcl_conversions )

install(TARGETS
        ${PROJECT_NAME}
        DESTINATION lib/${PROJECT_NAME}
        )

include_directories(include/)

install(TARGETS
${PROJECT_NAME}
DESTINATION lib/${PROJECT_NAME})

target_link_libraries(kitti_simple_slam_tool
        Eigen3::Eigen
        ${PCL_LIBRARIES})

#ament_package()

#ament_package()
ament_auto_package(INSTALL_TO_SHARE
        launch
        param
)
