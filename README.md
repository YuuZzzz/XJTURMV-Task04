# ROS2 Driver for Hikvision MVS Cameras

这是一个为海康威视工业相机（基于MVS SDK）开发的ROS2 Humble功能包。它旨在提供一个稳定、易于使用的接口，用于在ROS2项目中获取图像并控制相机参数。

## 核心功能

*   自动发现并连接第一个找到的 GigE 或 USB3.0 相机。
*   支持断线后自动重连。
*   以 `sensor_msgs/msg/Image` 格式高速发布图像话题。
*   通过 ROS2 参数系统动态配置相机核心参数。

## 依赖项

*   Ubuntu 22.04
*   ROS2 Humble Hawksbill
*   海康 MVS SDK (请从[海康机器人官网](https://www.hikrobotics.com/cn/machine-vision/service/download)下载适用于Linux的版本)

## 安装与编译

1.  **安装 MVS SDK**:
    请按照官方指引安装 MVS SDK。确保头文件和库文件位于 `/opt/MVS/` 目录下。

2.  **克隆并编译本功能包**:
    ```bash
    # 进入你的 ROS2 工作空间 src 目录
    cd ~/ros2_ws/src

    # 克隆本仓库 (未来你需要替换成你的 Git 仓库地址)
    git clone [你的Git仓库地址]

    # 回到工作空间根目录并编译
    cd ~/ros2_ws
    colcon build --packages-select hik_camera_ros2
    ```

## 如何使用

1.  **准备环境**:
    打开一个新终端，执行以下命令来准备环境。
    ```bash
    # 设置SDK库路径
    export LD_LIBRARY_PATH=/opt/MVS/lib/64:$LD_LIBRARY_PATH
    # 激活工作空间
    source ~/ros2_ws/install/setup.zsh
    ```

2.  **通过 Launch 文件启动**:
    这是推荐的启动方式，它会自动加载默认参数。
    ```bash
    ros2 launch hik_camera_ros2 hik_camera.launch.py
    ```

3.  **验证**:
    打开另一个准备好环境的终端，运行 Rviz2 并添加 `/image_raw` 话题进行查看。
    ```bash
    rviz2
    ```

## 接口说明

### 发布的话题

| 话题名 | 消息类型 | 描述 |
|---|---|---|
| `/image_raw` | `sensor_msgs/msg/Image` | 相机发布的原始图像数据。 |

### 可配置的参数

你可以通过 `ros2 param set` 命令或在 `config/params.yaml` 文件中修改以下参数：

| 参数名 | 类型 | 默认值 | 描述 |
|---|---|---|---|
| `exposure_time` | double | 8000.0 | 曝光时间，单位为微秒 (us)。 |
| `gain` | double | 5.0 | 增益值。 |
| `frame_rate` | double | 30.0 | 目标采集帧率 (fps)。 |
| `pixel_format` | string | "BGR8" | 像素格式。支持 "Mono8", "BGR8"。|

---
**作者**: [yuu from XJTU]
