cmake_minimum_required(VERSION 3.5)
project(rm_dh)

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

find_package(ament_cmake REQUIRED) 
find_package(rclcpp REQUIRED) 
find_package(std_msgs REQUIRED) 
find_package(joy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(jsoncpp REQUIRED)

include_directories(
  ${PROJECT_SOURCE_DIR}  
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}
  ${JsonCpp_INCLUDE_DIRS}
)
link_directories(${catkin_LIB_DIRS} ${PROJECT_SOURCE_DIR}/lib)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()
add_executable(rm_dh src/rm_dh.cpp)

target_link_libraries(${PROJECT_NAME}
  ${PROJECT_SOURCE_DIR}/lib/libRM_Service.so
  ${PROJECT_SOURCE_DIR}/lib/libRM_Service.so.1
  ${PROJECT_SOURCE_DIR}/lib/libRM_Service.so.1.0
  ${PROJECT_SOURCE_DIR}/lib/libRM_Service.so.1.0.0
  ${JsonCpp_LIBRARIES}
  /usr/lib/aarch64-linux-gnu/libjsoncpp.so.1.9.5
)

ament_target_dependencies(rm_dh rclcpp joy std_msgs)

install(TARGETS
rm_dh
  DESTINATION lib/${PROJECT_NAME})

ament_package()
