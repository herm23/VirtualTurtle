cmake_minimum_required(VERSION 3.8)
project(group28_assignament_1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# DEPENDENCIES
find_package(ament_cmake REQUIRED)  # build
find_package(apriltag_ros_tweaked REQUIRED)

find_package(rclcpp REQUIRED)       # component management
find_package(rclcpp_components REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(tf2_msgs REQUIRED)     # messages
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

######## CUSTOM INTERFACES ########
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TagDetection.msg"
  "srv/DetectTags.srv"
  "srv/DetectCircles.srv"
  DEPENDENCIES geometry_msgs
)
# This is crucial for using generated interfaces in the same package
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

######## TAG-DETECTOR LIBRARY/NODE ########
add_library(tag_detector SHARED src/tag_detector.cpp)
target_include_directories(tag_detector PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(tag_detector
  "rclcpp"
  "rclcpp_components"
  "tf2"
  "tf2_ros"
  "geometry_msgs"
)
target_link_libraries(tag_detector "${cpp_typesupport_target}")
rclcpp_components_register_node(tag_detector 
  PLUGIN "detection::TagDetector" 
  EXECUTABLE tag_detector_node
)


####### OTHERS #######
install(TARGETS
  tag_detector
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


# install(TARGETS
#   turtle
#   burrow
#   DESTINATION lib/${PROJECT_NAME}
# )

install(DIRECTORY include/ DESTINATION include)

####### LAUNCH #######
# install(DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME}
# )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
