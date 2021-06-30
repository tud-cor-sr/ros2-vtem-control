cmake_minimum_required(VERSION 3.5)
project(vtem_control)

# to we want to build with ROS?
option(BUILD_WITH_ROS2 "Do you want to build the package with ROS2 dependencies?" true)

# include directories
include_directories(include/vtem_control include/vtem_control/modbus)

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
endif()

find_package (Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

# message(STATUS ${CMAKE_MODULE_PATH})
# find_package(Modbus REQUIRED)
# https://github.com/AgileManufacturing/Avans/blob/master/cmake/FindModbus.cmake
find_package(PkgConfig)
pkg_check_modules(PC_MODBUS QUIET libmodbus)
set(MODBUS_DEFINITIONS ${PC_LIBMODBUS_CFLAGS_OTHER})
find_path(MODBUS_INCLUDE_DIRS modbus/modbus.h
          HINTS ${PC_LIBMODBUS_INCLUDEDIR} ${PC_LIBMODBUS_INCLUDE_DIRS}
          PATH_SUFFIXES modbus )
find_library(MODBUS_LIBRARIES NAMES modbus libmodbus
             HINTS ${PC_LIBMODBUS_LIBDIR} ${PC_LIBMODBUS_LIBRARY_DIRS} )
             include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set MODBUS_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(MODBUS "Could not find libmodbus" MODBUS_LIBRARIES MODBUS_INCLUDE_DIRS)

if(BUILD_WITH_ROS2)
# add custom ROS messages
  find_package(rosidl_default_generators REQUIRED)
  rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/InputPressures.msg"
  )
endif()

# add ROS executables
if(BUILD_WITH_ROS2)
  add_executable(pressure_regulator_node src/pressure_regulator_node.cpp)
  target_include_directories(pressure_regulator_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
endif()

# add other executables
add_executable(valve_test scripts/valve_test.cpp src/VtemControl.cpp)
target_include_directories(valve_test PRIVATE ${MODBUS_DIRS})
target_link_libraries(valve_test ${MODBUS_LIBRARIES} )

# add libraries
add_library(VtemControl SHARED src/VtemControl.cpp)
target_include_directories(VtemControl PRIVATE ${MODBUS_DIRS})
target_link_libraries(VtemControl -pthread ${MODBUS_LIBRARIES})

if(BUILD_WITH_ROS2)
  # Install Cpp executables
  install(TARGETS pressure_regulator_node DESTINATION lib/${PROJECT_NAME})
  
  # Install Python package
  # ament_python_install_package(${PROJECT_NAME})

  # Install Python executables
  install(PROGRAMS
  scripts/python_pub.py
  DESTINATION lib/${PROJECT_NAME}
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
