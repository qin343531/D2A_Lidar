cmake_minimum_required(VERSION 3.8)
project(lidar_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# 移除或注释下面这行，不使用强制 C++11
# add_definitions(-std=c++11)

# 包含 base 目录
include_directories(${CMAKE_SOURCE_DIR}/src/base)
# 包含 dep 目录
include_directories(${PROJECT_SOURCE_DIR}/include/lidar_ros2/dep)

# 如果有需要链接的第三方库，可以使用 link_directories 设置库路径
link_directories(${PROJECT_SOURCE_DIR}/include/lidar_ros2/dep/lib)

# 查找依赖项
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# 添加 base 目录下的所有源文件
set(BASE_SOURCES
    src/base/hchead.cpp
    src/base/HcSDK.cpp
    src/base/ReadParsePackage.cpp
    src/base/hclidar.cpp
    src/base/HC_serial.cpp
    src/base/lidar.cpp
)

# 添加 lidar_publisher 可执行文件
add_executable(lidar_publisher src/lidar_publisher.cpp ${BASE_SOURCES})

# 连接 ROS2 依赖库
ament_target_dependencies(lidar_publisher rclcpp sensor_msgs)

# 如果 dep 目录下有需要链接的库，使用 target_link_libraries 进行链接
# 例如，如果 dep 中有 `libexample.a`，你可以：
# target_link_libraries(lidar_publisher example)

# 安装可执行文件
install(TARGETS
  lidar_publisher
  DESTINATION lib/${PROJECT_NAME})

# 最后确保 amant_package() 被调用来正确处理包的构建
ament_package()

