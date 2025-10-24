# app_demo/ros_to_ros_demo/main.py
import sys
import os
import time

# --- 1. 导入 core_lib ---
# (.. -> app_demo, .. -> YoloHub)
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))
try:
    from core_lib.detector import Detector
    from core_lib.img_reader.ros_reader import ROSImageReader
    from core_lib.img_publisher.ros_image_publisher import ROSImagePublisher  # <--- 导入新工具
except ImportError as e:
    print(f"错误：无法导入模块: {e}")
    sys.exit(1)

# ==========================================================
# 2. 配置参数
# ==========================================================
MODEL_PATH = "../yolov8n.pt"
# TODO: 修改为您要订阅的图像话题
INPUT_TOPIC = "/camera/image_raw"
# TODO: 修改为您要发布的图像话题
OUTPUT_TOPIC = "/yolohub/annotated_image"

# ==========================================================
# 3. 初始化
# ==========================================================
try:
    print("正在初始化YOLO检测器...")
    detector = Detector(model_path=MODEL_PATH)

    print(f"正在订阅 ROS 话题: {INPUT_TOPIC}...")
    reader = ROSImageReader(topic_name=INPUT_TOPIC, node_name='yolohub_ros_pipeline')

    print(f"正在创建 ROS 发布器: {OUTPUT_TOPIC}...")
    publisher = ROSImagePublisher(topic_name=OUTPUT_TOPIC, node_name='yolohub_ros_pipeline')

    print("初始化完成，开始 'ROS-in, ROS-out' 循环...")
    print("这是一个无头(headless)节点，按 Ctrl+C 退出。")

except Exception as e:
    print(f"初始化失败: {e}")
    sys.exit(1)

# ==========================================================
# 4. 主检测循环
# ==========================================================
# 这个循环是“ROS-in, ROS-out”的核心
# 它从 'reader' 接收数据，处理后，交给 'publisher'
#
try:
    while not reader.rospy.is_shutdown():

        # 1. 获取最新帧
        frame = reader.read()

        if frame is None:
            time.sleep(0.01)  # 稍作等待，避免CPU空转
            continue

        # 2. 检测
        results = detector.detect(frame, verbose=False)

        # 3. 获取带标注的图像
        annotated_frame = results[0].plot()

        # 4. 发布带标注的图像
        publisher.publish(annotated_frame)

except KeyboardInterrupt:
    print("\n检测被用户中断 (Ctrl+C)。")

finally:
    # 5. 清理资源
    print("正在释放资源...")
    reader.release()
    print("程序退出。")