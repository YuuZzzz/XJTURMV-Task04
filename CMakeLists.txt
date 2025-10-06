# CMake 最低版本要求
cmake_minimum_required(VERSION 3.8)
# 项目名称
project(hik_camera_ros2)

# 查找 ROS2 的核心依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)

# 查找并配置海康 MVS SDK 
# 定义头文件搜索路径
include_directories(/opt/MVS/include)
# 定义库文件搜索路径
link_directories(/opt/MVS/lib/64)
# 打印状态信息，方便调试
message(STATUS "Hikvision SDK Include Path: /opt/MVS/include")
message(STATUS "Hikvision SDK Library Path: /opt/MVS/lib/64")

# 添加要编译的可执行文件
# 创建一个名为 hik_camera_node.cpp 的源文件
add_executable(hik_camera_node src/hik_camera_node.cpp)

# 为节点链接 ROS2 的依赖库
ament_target_dependencies(hik_camera_node
  "rclcpp"
  "image_transport"
  "sensor_msgs"
)

# 为节点链接海康 SDK 的核心库 
target_link_libraries(hik_camera_node 
    MvCameraControl # 对应 libMvCameraControl.so
)

# 安装规则，将编译好的节点安装到 ROS2 环境中
install(TARGETS hik_camera_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装 launch 和 config 文件
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)


# Install the environment hook script
install(
  FILES hik_camera_ros2.sh
  DESTINATION share/${PROJECT_NAME}
)

# ament 打包命令
ament_package()
