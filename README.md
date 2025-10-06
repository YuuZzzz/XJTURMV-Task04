# ROS2 Driver for Hikvision MVS Cameras

这是XJTURMV-Task04的任务结果，为了方便观看，已经将.zip文件中内容逐一放到main中，若要下载到本地，可以直接下载.zip文件，里面包含了本次任务的全部成果。

## 核心功能

*   自动发现并连接第一个找到的 GigE 或 USB3.0 相机。
*   支持断线后自动重连。
*   以 `sensor_msgs/msg/Image` 格式高速发布图像话题。
*   通过 ROS2 参数系统动态配置相机核心参数。

## 依赖项

*   Ubuntu 22.04
*   ROS2 Humble Hawksbill
*   海康 MVS SDK

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
    可以启动所有节点，包括Rviz2。
    ```bash
    ros2 launch hik_camera_ros2 hik_camera.launch.py
    ```

3.  **读取实际帧率**:
    打开另一个准备好环境的终端，可以读取当前实际帧率。
    ```bash
    ros2 topic hz  /image_raw
    ```

## 接口说明

### 发布的话题

| 话题名 | 消息类型 | 描述 |
|---|---|---|
| `/image_raw` | `sensor_msgs/msg/Image` | 相机发布的原始图像数据。 |

### 可配置的参数

可以通过 `ros2 param set` 命令或在 `config/params.yaml` 文件中修改以下参数：

| 参数名 | 类型 | 默认值 | 描述 |
|---|---|---|---|
| `exposure_time` | double | 8000.0 | 曝光时间，单位为微秒 (us)。 |
| `gain` | double | 5.0 | 增益值。 |
| `frame_rate` | double | 200.0 | 目标采集帧率 (fps)。 |
| `pixel_format` | string | "Mono8" | 像素格式。支持 "Mono8", "BGR8"。|

---
**作者**: yuu from XJTU
