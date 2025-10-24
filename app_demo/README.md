# YoloHub 应用示例 (app_demo)

这个目录包含了使用 `core_lib` 核心库的各种应用示例。

这些 Demo 展示了如何从不同的图像源（摄像头、RTSP、ROS）读取数据，并进行单线程或多线程的实时YOLO检测，以及如何将结果输出到本地窗口、ROS话题或远程Web服务器。

## 示例列表

### 1. 本地显示 Demos (Local Display)

这类 Demo 的**输出**是本地的 `cv2.imshow` 窗口，用于即时查看检测结果。

#### 1.1 单线程 (Simple)

按顺序执行 `读取 -> 检测 -> 显示`。适合离线处理或对实时性要求不高的场景。

* **[Simple Camera Demo](./simple_camera_demo/main.py)**
    * **功能**: 从本地USB摄像头读取图像，逐帧检测并实时显示结果。
    * **模式**: `CameraReader` (单线程)。
* **[Simple RTSP Demo](./simple_rtsp_demo/main.py)**
    * **功能**: 从RTSP网络视频流读取图像，逐帧检测并实时显示结果。
    * **模式**: `RTSPReader` (单线程)。

#### 1.2 多线程 (Multi-Threaded) - 推荐

采用“生产者-消费者”模式，将I/O读取放入独立线程，主线程专心检测。**这是实时应用的最佳实践**。

* **[Multi-Thread Camera Demo](./multi_thread_camera_demo/main.py)**
    * **功能**: 在“生产者”线程中高速读取USB摄像头，主线程（“消费者”）获取最新帧进行检测和显示。
    * **模式**: `CameraReader` + `ThreadedStream` (多线程)。
* **[Multi-Thread RTSP Demo](./multi_thread_rtsp_demo/main.py)**
    * **功能**: 在“生产者”线程中高速读取RTSP流，主线程（“消费者”）获取最新帧进行检测和显示。
    * **模式**: `RTSPReader` + `ThreadedStream` (多线程)。

#### 1.3 ROS -> 本地显示

* **[Multi-Thread ROS Demo](./multi_thread_ros_demo/main.py)** 
    * **功能**: 订阅一个ROS图像话题，对接收到的图像进行检测并显示。
    * **模式**: `ROSImageReader` (模块内部已实现多线程)。

---

### 2. ROS 节点 Demos (ROS-in -> ROS-out)

这类 Demo 作为ROS系统中的一个处理节点，**输入和输出都是ROS话题**。

* **[ROS to ROS Demo](./ros_to_ros_demo/main.py)**
    * **功能**: 订阅一个ROS图像话题，检测后，将**带标注的图像**发布到另一个ROS图像话题。
    * **模式**: `ROSImageReader` (输入) + `ROSImagePublisher` (输出)。

---

### 3. 远程仪表盘 Demos (HTTP 输出)

这类 Demo 作为**客户端**，将检测结果发送到**中央Web服务器**（即 `host_web_server` 项目），用于多设备集中监控。

* **[Camera to HTTP Demo](./camera_to_http_demo/main.py)**
    * **功能**: (多线程) 读取USB摄像头，检测后，将带标注的图像流发布到Web服务器的仪表盘。
    * **模式**: `CameraReader` + `ThreadedStream` (输入) + `HttpPublisher` (输出)。

* **[ROS to HTTP Demo](./ros_to_http_demo/main.py)**
    * **功能**: (无头节点) 订阅ROS图像话题，检测后，将带标注的图像流发布到Web服务器的仪表盘。
    * **模式**: `ROSImageReader` (输入) + `HttpPublisher` (输出)。