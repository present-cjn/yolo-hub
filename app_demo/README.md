# YoloHub 应用示例 (app_demo)

这个目录包含了使用 `core_lib` 核心库的各种应用示例。

这些 Demo 展示了如何从不同的图像源（摄像头、RTSP、ROS）读取数据，并进行单线程或多线程的实时YOLO检测。

## 示例列表

### 1. 单线程 (Simple Demos)

这类 Demo 使用简单直接的循环，按顺序执行 `读取 -> 检测 -> 显示`。适合用于离线处理或对实时性要求不高的场景。

* **[Simple Camera Demo](./simple_camera_demo/main.py)**
    * **功能**: 从本地USB摄像头读取图像，逐帧检测并实时显示结果。
    * **模式**: 单线程。
    * **核心模块**: `CameraReader`

* **[Simple RTSP Demo](./simple_rtsp_demo/main.py)**
    * **功能**: 从RTSP网络视频流读取图像，逐帧检测并实时显示结果。
    * **模式**: 单线程。
    * **核心模块**: `RTSPReader`

* **[Simple ROS Demo](./simple_ros_demo/main.py)**
    * **功能**: 订阅一个ROS图像话题 (`sensor_msgs/Image`)，对接收到的图像进行检测并显示。
    * **模式**: *伪*单线程（主循环是单线程的，但ROS订阅本身在后台是多线程的）。
    * **核心模块**: `ROSImageReader`

---

### 2. 多线程 (Multi-Thread Demos)

这类 Demo 采用了“生产者-消费者”模式，将耗时的I/O读取（如摄像头）放到一个独立的线程中，主线程则专心于YOLO检测。

**这是所有实时应用（如监控、机器人）的推荐实践**，它可以保证最低的延迟和最高的吞吐量。

* **[Multi-Thread Camera Demo](./multi_thread_camera_demo/main.py)**
    * **功能**: 在一个“生产者”线程中高速读取USB摄像头，主线程（“消费者”）从队列中获取最新帧进行检测和显示。
    * **模式**: 多线程（生产者-消费者）。
    * **核心模块**: `CameraReader` + `ThreadedStream`

* **[Multi-Thread RTSP Demo](./multi_thread_rtsp_demo/main.py)**
    * **功能**: 在一个“生产者”线程中高速读取RTSP流，主线程（“消费者”）获取最新帧进行检测和显示。
    * **模式**: 多线程（生产者-消费者）。
    * **核心模块**: `RTSPReader` + `ThreadedStream`

* **[Multi-Thread ROS Demo](./multi_thread_ros_demo/main.py)**
    * **功能**: 同 `Simple ROS Demo`。
    * **说明**: `ROSImageReader` 模块在设计上**内部已经实现了多线程**（ROS回调在后台线程运行），因此它天生就是“生产者-消费者”模式，无需额外使用 `ThreadedStream` 包装。
