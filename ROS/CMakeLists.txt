cmake_minimum_required(VERSION 3.8)
project(roi_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rcl_interfaces REQUIRED)
include_directories(
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${rcl_interfaces_INCLUDE_DIRS}
)

#Common Module/Ros Library Files
include_directories(lib) #lib library
file(GLOB lib_fls "../lib/*.cpp" "../lib/*.h" "../lib/*.hpp")
set(CROSSLIBS ${lib_fls})
add_library(lib ${CROSSLIBS})

if(BUILD_TESTING) #Whut this do? Copywrite checking ig.
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

#Generate All Interface Files
file(GLOB msg RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "msg/*.msg")
file(GLOB srv RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "srv/*.srv")
file(GLOB action RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "action/*.action")
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg}
  ${srv}
  ${action}
)

#Target Internal Interface Files
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)

#------ NODE EXECUTABLES ------

#Transport Agent
add_executable(base src/base.cpp src/base.h) #start executable
target_link_libraries(base lib) #link lib
target_link_libraries(base ${cpp_typesupport_target}) #link internal interface

ament_target_dependencies(base rclcpp std_msgs rcl_interfaces ) #amet linking

#------ NODE EXECUTABLES ------

#Remeber to install the executables in the install section
install(TARGETS
  base
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

#------ Python Compilation ------
#ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/transportAgent.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
