cmake_minimum_required(VERSION 3.8)
project(tape_navigation)


# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
 set(CMAKE_CXX_STANDARD 17)
endif()


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
 add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS highgui imgcodecs imgproc videoio)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)


include_directories(include)




# add the subscriber example
add_executable(tape_detection_node src/tape_detection.cpp)
target_link_libraries(tape_detection_node
 cv_bridge::cv_bridge
 image_transport::image_transport
 opencv_highgui
 rclcpp::rclcpp
)


# Install executables
install(
 TARGETS tape_detection_node
 RUNTIME DESTINATION lib/${PROJECT_NAME}
)


# Install include directories
# install(
#  DIRECTORY include/
#  DESTINATION include/${PROJECT_NAME}
# )


ament_export_include_directories(include)
ament_export_dependencies(cv_bridge image_transport pluginlib rosidl_default_runtime rclcpp sensor_msgs std_msgs)


if(BUILD_TESTING)
 find_package(ament_lint_auto REQUIRED)
 ament_lint_auto_find_test_dependencies()
endif()

ament_package()
