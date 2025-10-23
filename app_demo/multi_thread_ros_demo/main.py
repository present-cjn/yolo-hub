# app_demo/multi_thread_ros_demo/main.py
import cv2
import sys
import os
import time

# --- 1. 导入 core_lib ---
# (.. -> app_demo, .. -> YoloHub)
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))
try:
    from core_lib.detector import Detector
    from core_lib.img_reader.ros_reader import ROSImageReader  # <--- 导入ROSReader
except ImportError as e:
    print(f"错误：无法导入模块: {e}")
    sys.exit(1)

# ==========================================================
# 2. 配置参数
# ==========================================================
MODEL_PATH = "../yolov8n.pt"
ROS_IMAGE_TOPIC = "/camera/image_raw"  # TODO: 修改为您要订阅的图像话题
WINDOW_NAME = "YoloHub Multi-Thread ROS Demo"

# ==========================================================
# 3. 初始化
# ==========================================================
try:
    print("正在初始化YOLO检测器...")
    detector = Detector(model_path=MODEL_PATH)

    print(f"正在订阅 ROS 话题: {ROS_IMAGE_TOPIC}...")
    # ROSImageReader 内部的 rospy.Subscriber 已经
    # 在一个独立的ROS线程中运行回调 (生产者线程)
    reader = ROSImageReader(topic_name=ROS_IMAGE_TOPIC)

    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    print("初始化完成，等待ROS图像消息...")
    print("按 'q' 键退出。")

except Exception as e:
    print(f"初始化失败: {e}")
    sys.exit(1)

# ==========================================================
# 4. 主检测循环 (消费者线程)
# ==========================================================
#
# 注意：我们不需要 ThreadedStream 包装器，
# 因为 ROSImageReader 已经通过回调在后台线程中接收图像了。
# 这个主循环本身就是“消费者”线程。
#
try:
    # 循环直到ROS关闭 (例如 Ctrl+C)
    while not reader.rospy.is_shutdown():

        # 1. 从内部缓冲区获取最新一帧 (非阻塞)
        frame = reader.read()

        if frame is None:
            # print("等待第一帧...")
            time.sleep(0.1)  # 稍作等待，避免CPU空转
            continue

        # 2. 检测
        results = detector.detect(frame, verbose=False)

        # 3. 获取带标注的图像
        annotated_frame = results[0].plot()

        # 4. 显示结果
        cv2.imshow(WINDOW_NAME, annotated_frame)

        # 检查退出键
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("检测到 'q' 键，正在退出...")
            break

except KeyboardInterrupt:
    print("\n检测被用户中断 (Ctrl+C)。")

finally:
    # 5. 清理资源
    print("正在释放资源并关闭窗口...")
    reader.release()
    cv2.destroyAllWindows()
    print("程序退出。")