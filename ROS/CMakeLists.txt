cmake_minimum_required(VERSION 3.8)
project(roi_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
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

#Local Libraries
SET(BUILD_STATIC_LIBS ON) 
add_subdirectory(libs/libSocket)

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
add_executable(transportAgent src/transportAgent.cpp src/transportAgent.h) #start executable
target_link_libraries(transportAgent lib) #link lib
target_link_libraries(transportAgent srclib) #link srclib
target_link_libraries(transportAgent ${cpp_typesupport_target}) #link internal interface
target_link_libraries(transportAgent libSocket++_int)

ament_target_dependencies(transportAgent rclcpp std_msgs rcl_interfaces ) #amet linking

#------ NODE EXECUTABLES ------

#Remeber to install the executables in the install section
install(TARGETS
  transportAgent
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
