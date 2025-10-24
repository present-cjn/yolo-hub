# YoloHub/app_demo/http_camera_demo/main.py
import cv2
import sys
import os

# --- 1. 导入 core_lib ---
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))
try:
    from core_lib.detector import Detector
    from core_lib.img_reader.camera_reader import CameraReader
    from core_lib.threaded_stream import ThreadedStream
    from core_lib.publisher.http_publisher import HttpPublisher  # <--- 导入新模块
except ImportError:
    print("错误：无法从 core_lib 导入模块。")
    sys.exit(1)

# ==========================================================
# 2. 配置参数
# ==========================================================
MODEL_PATH = "../yolov8n.pt"
CAMERA_ID = 0
WINDOW_NAME = "YoloHub HTTP Demo (Client)"
QUEUE_SIZE = 1

# TODO: ！！重要！！
# 将 '127.0.0.1' 替换为您运行“服务器”的主机IP地址
# 如果在同一台机器上测试，使用 '127.0.0.1' 即可
SERVER_IP = "192.168.3.101"
SERVER_URL = f"http://{SERVER_IP}:5000"

# 当您在第二台设备上运行此脚本时，请把它改成 "device_02"
DEVICE_ID = "device_01"


# ==========================================================
# 3. 初始化
# ==========================================================
try:
    print("正在初始化检测器...")
    detector = Detector(model_path=MODEL_PATH)

    print(f"正在打开摄像头 {CAMERA_ID}...")
    reader = CameraReader(camera_id=CAMERA_ID)
    threaded_reader = ThreadedStream(reader, queue_size=QUEUE_SIZE).start()

    print(f"正在初始化HTTP发布器，目标: {SERVER_URL}")
    publisher = HttpPublisher(server_url=SERVER_URL, device_id=DEVICE_ID)
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    print("初始化完成，按 'q' 键退出。")

except Exception as e:
    print(f"初始化时发生错误: {e}")
    sys.exit(1)

# ==========================================================
# 4. 主检测循环 (消费者)
# ==========================================================
try:
    while threaded_reader.more():
        frame = threaded_reader.read()
        results = detector.detect(frame, verbose=False)
        annotated_frame = results[0].plot()

        cv2.imshow(WINDOW_NAME, annotated_frame)
        publisher.publish(annotated_frame)  # 发布

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
except KeyboardInterrupt:
    print("\n检测被用户中断 (Ctrl+C)。")
finally:
    # 5. 清理资源
    print("正在停止读取线程并关闭窗口...")
    threaded_reader.stop()
    cv2.destroyAllWindows()
    print("客户端程序退出。")