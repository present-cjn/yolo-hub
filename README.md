# YoloHub

YoloHub 是一个模块化、可扩展的 Python 框架，用于快速构建和部署实时的 YOLOv8 检测流水线。

它的设计核心是将复杂的I/O操作（如读取RTSP流、订阅ROS话题、发布到HTTP服务器）封装成可复用的组件，让您可以专注于检测逻辑本身。本项目非常适合在多个边缘设备（如 Jetson Orin NX, 笔记本电脑）上部署，并与ROS或Web仪表盘集成。

## 核心特性

* **模块化 `core_lib`**: 将所有可复用的逻辑（检测器、读取器、发布器）沉淀为核心库。
* **多输入支持**: 预置了`CameraReader`, `RTSPReader`, 和 `ROSImageReader`。
* **多输出支持**: 预置了`ROSImagePublisher` (发布ROS图像) 和 `HttpPublisher` (发布到Web仪表盘)。
* **实时性优化**: 内置 `ThreadedStream` 包装器，可一键将任何I/O读取器转换为高性能的多线程“生产者-消费者”模式，防止YOLO检测阻塞视频流。

## 项目结构

```
YoloHub/
├── core_lib/               # <-- 核心可复用库 (detector, img_reader, publisher)
│   ├── detector.py
│   ├── threaded_stream.py
│   ├── img_reader/
│   │   ├── camera_reader.py
│   │   ├── rtsp_reader.py
│   │   └── ros_reader.py
│   └── publisher/
│       ├── ros_image_publisher.py
│       └── http_publisher.py
│
├── app_demo/               # <-- "配方"集合：展示如何组装 core_lib 的示例
│   ├── simple_camera_demo/
│   ├── multi_thread_rtsp_demo/
│   ├── ros_to_http_demo/
│   └── README.md           # <-- (包含所有Demo的详细列表!)
│
├── requirements.txt        # <-- 本项目的主要 Python 依赖
├── README.md               # <-- 您正在阅读的
└── ...                     # <-- 您未来的应用脚本 (例如: run_orin_app.py)
```

## 1. 安装

在您的设备（例如 Orin NX 或 笔记本电脑）上设置 YoloHub：

1.  **克隆仓库**

2.  **创建并激活虚拟环境 (推荐)**
    ```bash
    python3 -m venv venv
    source venv/bin/activate
    ```

3.  **安装 Python 依赖**
    ```bash
    pip install -r requirements.txt
    ```

4.  **(若需要) 安装 ROS 依赖**
    如果您需要使用 ROS 相关的 Demo，请确保您的 ROS 环境已激活，并已安装 `cv_bridge`：
    ```bash
    # 示例 (for Noetic)
    source /opt/ros/noetic/setup.bash
    sudo apt-get install ros-noetic-cv-bridge
    ```

## 2. 如何运行一个应用 (推荐的工作流)

`app_demo` 目录是用来学习的“配方”，**不推荐**直接在里面修改代码。

推荐的工作流是“挑选”一个 Demo，并将其提升为根目录下的一个正式应用脚本：

1.  **挑选 Demo**:
    浏览 `app_demo/` 目录 (和它的 `README.md`)，找到最接近您需求的 Demo。例如，您想在 Orin 上运行 "ROS 输入 -> HTTP 输出" 的应用。

2.  **复制到根目录**:
    将该 Demo 的 `main.py` 复制到项目的**根目录** (`yolo-hub/`)，并给它一个清晰的名字。
    ```bash
    # (在 YoloHub/ 根目录下)
    cp app_demo/ros_to_http_demo/main.py ./