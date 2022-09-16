cmake_minimum_required(VERSION 3.8)
project(opencv_apps)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

set(node_dependencies
  rclcpp
  rclcpp_components
  rcutils
  std_msgs
  image_transport
  cv_bridge
  rmw_implementation
)

## Declare the opencv_apps_ros library
add_library(${PROJECT_NAME} SHARED
    # src/adding_images.cpp
    # src/camshift.cpp
    # src/color_filter.cpp
    # src/contour_moments.cpp
    # src/convex_hull.cpp
    # src/corner_harris.cpp
    # src/discrete_fourier_transform.cpp
    src/edge_detection.cpp
    # src/equalize_histogram.cpp
    # src/face_detection.cpp
    # src/face_recognition.cpp
    # src/fback_flow.cpp
    # src/find_contours.cpp
    # src/general_contours.cpp
    # src/goodfeature_track.cpp
    # src/hough_circles.cpp
    # src/hough_lines.cpp
    # src/lk_flow.cpp
    src/node.cpp
    # src/morphology.cpp
    # src/people.cpp
    # src/phase_corr.cpp
    # src/pyramids.cpp
    # src/segment_objects.cpp
    # src/simple_compressed_example.cpp
    # src/simple_example.cpp
    # src/simple_flow.cpp
    # src/smoothing.cpp
    # src/threshold.cpp
    # src/watershed_segmentation.cpp
)

ament_target_dependencies(${PROJECT_NAME} ${node_dependencies})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES})
# prevent pluginlib from using boost
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")


target_link_libraries(${PROJECT_NAME})
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "opencv_apps::EdgeDetection"
  EXECUTABLE edge_detection
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include/
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(FILES plugins_description.xml
  DESTINATION share
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_plugins)
pluginlib_export_plugin_description_file(${PROJECT_NAME} plugins_description.xml)

ament_package()
