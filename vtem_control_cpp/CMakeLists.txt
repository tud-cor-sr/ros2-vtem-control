cmake_minimum_required(VERSION 3.5)
project(vtem_control_cpp)

# to we want to build with ROS?
option(BUILD_WITH_ROS2 "Do you want to build the package with ROS2 dependencies?" true)

# set cmake module path
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# include directories
include_directories(include/vtem_control_cpp)

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

if(BUILD_WITH_ROS2)
  # find ROS dependencies
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(pneumatic_actuation_msgs REQUIRED) 
endif()

find_package (Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

# message(STATUS ${CMAKE_MODULE_PATH})
find_package(Modbus REQUIRED)
include_directories(${MODBUS_INCLUDE_DIR})

# add ROS executables
if(BUILD_WITH_ROS2)
  # input pressures subscriber node
  add_executable(input_pressures_sub_node src/input_pressures_sub_node.cpp src/VtemControl.cpp)
  ament_target_dependencies(input_pressures_sub_node pneumatic_actuation_msgs rclcpp)
  target_include_directories(input_pressures_sub_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  target_link_libraries(input_pressures_sub_node ${MODBUS_LIBRARIES})

  # output pressures subscriber node
  add_executable(output_pressures_pub_node src/output_pressures_pub_node.cpp src/VtemControl.cpp)
  ament_target_dependencies(output_pressures_pub_node pneumatic_actuation_msgs rclcpp)
  target_include_directories(output_pressures_pub_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  target_link_libraries(output_pressures_pub_node ${MODBUS_LIBRARIES})
endif()

# add other executables
add_executable(valve_test scripts/valve_test.cpp src/VtemControl.cpp)
target_include_directories(valve_test PRIVATE)
target_link_libraries(valve_test ${MODBUS_LIBRARIES} )

# add libraries
add_library(VtemControl SHARED src/VtemControl.cpp)
target_include_directories(VtemControl PRIVATE)
target_link_libraries(VtemControl -pthread ${MODBUS_LIBRARIES})

if(BUILD_WITH_ROS2)
  # Install Cpp executables
  install(TARGETS input_pressures_sub_node DESTINATION lib/${PROJECT_NAME})
  install(TARGETS output_pressures_pub_node DESTINATION lib/${PROJECT_NAME})
  
  # Install Python package
  # ament_python_install_package(${PROJECT_NAME})

  # Install Python executables
  # install(PROGRAMS
  # scripts/python_pub_node.py
  # DESTINATION lib/${PROJECT_NAME}
  # )

  # Install launch files
  # Install launch files
  install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
  )
endif()

# install other executables and libraries
install(TARGETS VtemControl DESTINATION lib/${PROJECT_NAME})
install(TARGETS valve_test DESTINATION lib/${PROJECT_NAME})

if(BUILD_WITH_ROS2 AND BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

if(BUILD_WITH_ROS2)
  ament_package()
endif()
