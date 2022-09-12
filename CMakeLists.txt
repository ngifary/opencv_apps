cmake_minimum_required(VERSION 3.5)
project(opencv_apps)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fPIC)
endif()

## Find ROS package dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(OpenCV REQUIRED)

ament_export_dependencies(
    message_runtime 
    rclcpp
    rclcpp_components
    sensor_msgs 
    std_msgs 
    std_srvs
    rclcpp
    rosbag2
)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

## # Base node
add_library(opencv_apps_node src/component/node.cpp)
ament_target_dependencies(opencv_apps_node
  "rclcpp"
  "image_transport"
)
target_link_libraries(opencv_apps_node ${OpenCV_LIBRARIES})

# ## Declare the opencv_apps_ros library
# add_library(opencv_apps_ros
#     # src/component/adding_images.cpp
#     # src/component/camshift.cpp
#     # src/component/color_filter.cpp
#     # src/component/contour_moments.cpp
#     # src/component/convex_hull.cpp
#     # src/component/corner_harris.cpp
#     # src/component/discrete_fourier_transform.cpp
#     src/component/edge_detection.cpp
#     # src/component/equalize_histogram.cpp
#     # src/component/face_detection.cpp
#     # src/component/face_recognition.cpp
#     # src/component/fback_flow.cpp
#     # src/component/find_contours.cpp
#     # src/component/general_contours.cpp
#     # src/component/goodfeature_track.cpp
#     # src/component/hough_circles.cpp
#     # src/component/hough_lines.cpp
#     # src/component/lk_flow.cpp
#     # src/component/morphology.cpp
#     # src/component/people.cpp
#     # src/component/phase_corr.cpp
#     # src/component/pyramids.cpp
#     # src/component/segment_objects.cpp
#     # src/component/simple_compressed_example.cpp
#     # src/component/simple_example.cpp
#     # src/component/simple_flow.cpp
#     # src/component/smoothing.cpp
#     # src/component/threshold.cpp
#     # src/component/watershed_segmentation.cpp
# )

# ament_target_dependencies(opencv_apps_ros
#   "rclcpp"
#   "rclcpp_components"
#   "rmw_implementation"
# )
# target_link_libraries(opencv_apps_ros opencv_apps_node)
# ament_export_libraries(opencv_apps_ros)

# # Create component for edge detection
# add_library(opencv_apps_edge_detection SHARED
#     src/component/edge_detection.cpp
# )
# target_link_libraries(opencv_apps_edge_detection opencv_apps_ros)
# rclcpp_components_register_node(opencv_apps_edge_detection PLUGIN
#   PLUGIN "opencv_apps::EdgeDetection"
#   EXECUTABLE opencv_apps_edge_detection_node
# )
# install(TARGETS
#     opencv_apps_edge_detection
#     ARCHIVE DESTINATION lib
#     LIBRARY DESTINATION lib
#     RUNTIME DESTINATION bin
# )

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif(BUILD_TESTING)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS
  opencv_apps_node
#   opencv_apps_ros
  EXPORT export_opencv_apps
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(opencv_apps_node)
ament_export_dependencies(${dependencies})
ament_export_targets(export_opencv_apps HAS_LIBRARY_TARGET)
ament_package()