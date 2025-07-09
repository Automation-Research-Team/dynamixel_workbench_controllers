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
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(dynamixel_workbench_msgs REQUIRED)
find_package(dynamixel_workbench_toolbox REQUIRED)
find_package(cmake_modules REQUIRED)

find_package(Eigen3 REQUIRED)

###########
#  Build  #
###########
include_directories(
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>
)

add_executable(
  ${PROJECT_NAME}
    src/dynamixel_workbench_controllers.cpp
    src/trajectory_generator.cpp
)

target_link_libraries(
  ${PROJECT_NAME}
    ${Eigen3_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
)

add_executable(find_dynamixel src/find_dynamixel.cpp)
add_dependencies(find_dynamixel ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(find_dynamixel ${catkin_LIBRARIES})

###############
##  Install  ##
###############
install(
  TARGETS
    ${PROJECT_NAME} find_dynamixel
  EXPORT   export_${PROJECT_NAME}
  LIBRARY  DESTINATION lib
  ARCHIVE  DESTINATION lib
  RUNTIME  DESTINATION bin
  INCLUDES DESTINATION include
)
