cmake_minimum_required(VERSION 3.8)
project(object_detection)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

cmake_policy(SET CMP0074 NEW)

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL 1.12 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(octomap REQUIRED)
find_package(backward_ros REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} include)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(filter_single_frame example/filter_single_frame.cpp)
add_executable(filter_multi_frames example/filter_multi_frames.cpp)
add_executable(dbscan example/dbscan.cpp)
add_executable(test_icp example/test_icp.cpp)

target_link_libraries(filter_single_frame
  ${Boost_SYSTEM_LIBRARY}
  ${PCL_LIBRARIES}
)
target_link_libraries(filter_multi_frames
  ${Boost_SYSTEM_LIBRARY}
  ${PCL_LIBRARIES}
)
target_link_libraries(dbscan
  ${Boost_SYSTEM_LIBRARY}
  ${PCL_LIBRARIES}
)
target_link_libraries(test_icp
  ${Boost_SYSTEM_LIBRARY}
  ${PCL_LIBRARIES}
)

ament_target_dependencies(filter_single_frame
rclcpp PCL pcl_conversions pcl_ros)
ament_target_dependencies(filter_multi_frames
rclcpp PCL pcl_conversions pcl_ros octomap visualization_msgs)
ament_target_dependencies(dbscan
rclcpp PCL pcl_conversions pcl_ros visualization_msgs)
ament_target_dependencies(test_icp
rclcpp PCL pcl_conversions pcl_ros octomap visualization_msgs)

install(TARGETS
  filter_single_frame
  filter_multi_frames
  dbscan
  test_icp
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch rviz map include
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()

