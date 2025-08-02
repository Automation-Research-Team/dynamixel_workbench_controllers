cmake_minimum_required(VERSION 3.8)
project(dynamixel_workbench_controllers)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(PkgConfig)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(dynamixel_workbench_msgs REQUIRED)
find_package(dynamixel_workbench_toolbox REQUIRED)
find_package(ddynamic_reconfigure2 REQUIRED)

find_package(Eigen3 REQUIRED)

###########
#  Build  #
###########
add_library(
  ${PROJECT_NAME} SHARED
    src/dynamixel_workbench_controllers.cpp
    src/trajectory_generator.cpp
)

target_include_directories(
  ${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_definitions(
  ${PROJECT_NAME} PRIVATE
    COMPOSITION_BUILDING_DLL
)

ament_target_dependencies(
  ${PROJECT_NAME}
    rclcpp
    rclcpp_components
    sensor_msgs
    geometry_msgs
    trajectory_msgs
    dynamixel_workbench_msgs
    dynamixel_workbench_toolbox
    ddynamic_reconfigure2
)

target_link_libraries(
  ${PROJECT_NAME}
    ${Eigen3_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
)

rclcpp_components_register_node(
  ${PROJECT_NAME}
    PLUGIN     ${PROJECT_NAME}::DynamixelController
    EXECUTABLE ${PROJECT_NAME}_node
)

#############
#  Install  #
#############
install(
  TARGETS  ${PROJECT_NAME} #find_dynamixel
  EXPORT   export_${PROJECT_NAME}
  LIBRARY  DESTINATION lib
  ARCHIVE  DESTINATION lib
  RUNTIME  DESTINATION bin
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}/
)

#############
#  Exports  #
#############
ament_export_libraries(${PROJECT_NAME})

##########
#  test  #
##########
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

ament_package()
