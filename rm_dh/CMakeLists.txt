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
find_package(sensor_msgs REQUIRED)  # 新增：添加sensor_msgs依赖 *************
find_package(rosidl_default_generators REQUIRED)  # 新增：接口生成工具 *********

include_directories(
  ${PROJECT_SOURCE_DIR}  
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}
  ${JsonCpp_INCLUDE_DIRS}
)
link_directories(${catkin_LIB_DIRS} ${PROJECT_SOURCE_DIR}/lib)

# 新增：配置服务接口 **********************************************************
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/JointCommand.srv"  # 定义服务接口文件
  DEPENDENCIES sensor_msgs  # 指定依赖的消息包
)

install(
  # DIRECTORY srv  # 新增：安装服务定义文件 *************************************
  DIRECTORY launch
  DESTINATION share/rm_dh_node
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()
add_executable(rm_dh_node src/rm_dh.cpp)

target_link_libraries(rm_dh_node
  ${PROJECT_SOURCE_DIR}/lib/libRM_Service.so
  ${PROJECT_SOURCE_DIR}/lib/libRM_Service.so.1
  ${PROJECT_SOURCE_DIR}/lib/libRM_Service.so.1.0
  ${PROJECT_SOURCE_DIR}/lib/libRM_Service.so.1.0.0
  ${JsonCpp_LIBRARIES}
  /usr/lib/aarch64-linux-gnu/libjsoncpp.so.1.9.5
)

ament_target_dependencies(rm_dh_node rclcpp joy std_msgs sensor_msgs)  # 添加sensor_msgs依赖

# 新增：链接生成的服务接口库
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(rm_dh_node "${cpp_typesupport_target}")

ament_target_dependencies(rm_dh_node rclcpp joy std_msgs)

install(TARGETS
  rm_dh_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
