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
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)

include_directories(
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${rcl_interfaces_INCLUDE_DIRS}
)

#Common Module/Ros Library Files
include_directories(lib) #lib library
file(GLOB lib_fls "../lib/*.cpp" "../lib/*.h" "../lib/*.hpp")
set(CROSSLIBS ${lib_fls})
add_library(lib STATIC ${CROSSLIBS})

if(BUILD_TESTING) #Unsure what this does
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

#oDrive Node
add_executable(oDrive src/ODrive.cpp src/ODrive.h src/base.h src/base.cpp) #start executable
target_link_libraries(oDrive lib) #link lib
target_link_libraries(oDrive ${cpp_typesupport_target}) #link internal interface

ament_target_dependencies(oDrive rclcpp std_msgs rcl_interfaces rclcpp_action rclcpp_components ) #amet linking

#Actuator Node
add_executable(Actuator src/Actuator.cpp src/Actuator.h src/base.h src/base.cpp) #start executable
target_link_libraries(Actuator lib) #link lib
target_link_libraries(Actuator ${cpp_typesupport_target}) #link internal interface

ament_target_dependencies(Actuator rclcpp std_msgs rcl_interfaces rclcpp_action rclcpp_components ) #amet linking
#------ NODE EXECUTABLES ------

#Remember to install the executables in the install section
install(TARGETS
  oDrive 
  Actuator
  DESTINATION lib/${PROJECT_NAME}
)

#------ Python Compilation ------
install(PROGRAMS
  scripts/transportAgent.py
  DESTINATION lib/${PROJECT_NAME})

install(FILES 
  scripts/netTransmitter.py
  scripts/netReceiver.py
  scripts/reliabilityManager.py
  DESTINATION lib/${PROJECT_NAME})


#-- Launch Files -- 
install(DIRECTORY
launch
DESTINATION share/${PROJECT_NAME}/
)


#-- Exports --
ament_export_dependencies(rosidl_default_runtime rosidl_typesupport_cpp rosidl_typesupport_c)
ament_package()
