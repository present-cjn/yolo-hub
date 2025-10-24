# app_demo/ros_to_http_demo/main.py
import sys
import os
import time

# --- 1. 导入 core_lib ---
# (.. -> app_demo, .. -> YoloHub)
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))
try:
    from core_lib.detector import Detector
    from core_lib.img_reader.ros_reader import ROSImageReader
    from core_lib.publisher.http_publisher import HttpPublisher
except ImportError as e:
    print(f"错误：无法从 core_lib 导入模块: {e}")
    sys.exit(1)

# ==========================================================
# 2. 配置参数
# ==========================================================
MODEL_PATH = "../yolov8n.pt"

# TODO: 1. 修改为您要订阅的ROS图像话题
INPUT_ROS_TOPIC = "/camera/image_raw"

# TODO: 2. 修改为您的主机(服务器)IP地址
SERVER_IP = "127.0.0.1"
SERVER_URL = f"http://{SERVER_IP}:5000"

# TODO: 3. 为这个设备设置一个唯一的ID
DEVICE_ID = "ros_device_01"

# ==========================================================
# 3. 初始化
# ==========================================================
try:
    print("正在初始化YOLO检测器...")
    detector = Detector(model_path=MODEL_PATH)

    # ROSImageReader 会自动初始化ROS节点
    print(f"正在订阅 ROS 话题: {INPUT_ROS_TOPIC}...")
    reader = ROSImageReader(topic_name=INPUT_ROS_TOPIC, node_name='yolohub_ros_to_http')

    print(f"正在初始化HTTP发布器，将上报至: {SERVER_URL}/upload/{DEVICE_ID}")
    publisher = HttpPublisher(server_url=SERVER_URL, device_id=DEVICE_ID)

    print("\n初始化完成。")
    print("这是一个无头(headless)节点，正在运行...")
    print("按 Ctrl+C 退出。")

except Exception as e:
    print(f"初始化失败: {e}")
    sys.exit(1)

# ==========================================================
# 4. 主检测循环
# ==========================================================
# 这个循环是“ROS-in, HTTP-out”的核心
# 它从 'reader' 接收数据，处理后，交给 'publisher'
#
try:
    while not reader.rospy.is_shutdown():

        # 1. 获取最新帧
        frame = reader.read()

        if frame is None:
            # 等待第一帧...
            time.sleep(0.1)
            continue

        # 2. 检测
        results = detector.detect(frame, verbose=False)

        # 3. 获取带标注的图像
        annotated_frame = results[0].plot()

        # 4. 发布带标注的图像
        # publisher.publish 会处理网络错误，不会让节点崩溃
        publisher.publish(annotated_frame)

        # (注意：没有 cv2.imshow 和 cv2.waitKey)

except KeyboardInterrupt:
    print("\n检测被用户中断 (Ctrl+C)。")

finally:
    # 5. 清理资源
    print("正在释放资源...")
    reader.release()
    print("程序退出。")