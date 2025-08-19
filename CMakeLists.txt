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

##################
#  Dependencies  #
##################
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

###########
#  Build  #
###########
ament_auto_add_library(
  ${PROJECT_NAME} SHARED
    src/dynamixel_workbench_controllers.cpp
    src/trajectory_generator.cpp
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
  TARGETS  ${PROJECT_NAME}
  EXPORT   export_${PROJECT_NAME}
  LIBRARY  DESTINATION lib
  ARCHIVE  DESTINATION lib
  RUNTIME  DESTINATION bin
)

install(
  DIRECTORY   launch config
  DESTINATION share/${PROJECT_NAME}/
)

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

ament_auto_package()
