cmake_minimum_required(VERSION 3.8)
project(roi_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
)

include_directories(lib) #lib library

file(GLOB lib_fls "../lib/*.cpp" "../lib/*.h" "../lib/*.hpp")
set(LIB_SRCS ${lib_fls})

include_directories(SBC) #src

file(GLOB src_fls "lib/*.cpp" "lib/*.h" "lib/*.hpp" "src/*.cpp" "src/*.h" "src/*.hpp" "lib/moduleVirtualizations/*.cpp" "lib/moduleVirtualizations/*.h" "lib/moduleVirtualizations/*.hpp")
set(SRC_SRCS ${src_fls})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/HeadLightState.msg"
  "srv/SetHeadLight.srv" #can you put wildcard here? I don't know why they wouldn't want you to do that
)

add_library(lib ${LIB_SRCS})
add_library(SBC ${SRC_SRCS})

add_executable(ROI src/main.cpp)
target_link_libraries(ROI lib)
target_link_libraries(ROI SBC)

ament_target_dependencies(ROI rclcpp std_msgs)

install(TARGETS
  ROI
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
