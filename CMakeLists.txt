cmake_minimum_required(VERSION 3.5)
project(localization)

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

include(cmake/YAML.cmake)
include(cmake/global_defination.cmake)
include(cmake/geographic.cmake)
include(cmake/glog.cmake)
include(cmake/PCL.cmake)
include(cmake/openmp.cmake)


#对应需要编译cpp的文件路径
file(GLOB_RECURSE ALL_SRCS "src/*.cpp")
file(GLOB_RECURSE MAIN_SRCS "src/*_main.cpp")
list(REMOVE_ITEM ALL_SRCS ${MAIN_SRCS})

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

include_directories(${PROJECT_SOURCE_DIR}/include/localization)

rosidl_generate_interfaces(${PROJECT_NAME} "msg/CloudInfo.msg" DEPENDENCIES std_msgs sensor_msgs)

add_executable(preprocess_data src/apps/preprocess_data_main.cpp ${ALL_SRCS})
ament_target_dependencies(preprocess_data rclcpp sensor_msgs nav_msgs)
rosidl_target_interfaces(preprocess_data ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_compile_options(preprocess_data PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(preprocess_data ${ALL_TARGET_LIBRARIES} ${OpenMP_CXX_FLAGS})

add_executable(front_end src/apps/front_end_main.cpp ${ALL_SRCS})
ament_target_dependencies(front_end rclcpp sensor_msgs nav_msgs)
rosidl_target_interfaces(front_end ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_compile_options(preprocess_data PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(front_end ${ALL_TARGET_LIBRARIES})

install(TARGETS
  preprocess_data
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  front_end
  DESTINATION lib/${PROJECT_NAME})


# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY rviz2
  DESTINATION share/${PROJECT_NAME}/
)


ament_package()
