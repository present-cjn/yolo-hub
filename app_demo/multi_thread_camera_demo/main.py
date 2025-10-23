# app_demo/multi_thread_camera_demo/main.py
import cv2
import sys
import os

# --- 1. 导入 core_lib ---
# 将项目根目录 (YoloHub) 添加到Python路径
# (.. -> app_demo, .. -> YoloHub)
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))
try:
    from core_lib.detector import Detector
    from core_lib.img_reader.camera_reader import CameraReader
    from core_lib.threaded_stream import ThreadedStream  # <--- 导入多线程包装器
except ImportError:
    print("错误：无法从 core_lib 导入模块。")
    print("请确保 core_lib 文件夹在 YoloHub 根目录下。")
    sys.exit(1)

# ==========================================================
# 2. 配置参数
# ==========================================================
# 模型路径 (位于上一级 app_demo 目录中)
MODEL_PATH = "../yolov8n.pt"
CAMERA_ID = 0
WINDOW_NAME = "YoloHub Multi-Thread Camera Demo"
QUEUE_SIZE = 1  # 缓冲区大小，1表示只保留最新帧，实现最佳实时性

# ==========================================================
# 3. 初始化
# ==========================================================
try:
    print("正在初始化YOLO检测器...")
    detector = Detector(model_path=MODEL_PATH)

    print(f"正在打开摄像头 {CAMERA_ID}...")
    # 1. 先创建原始的reader
    reader = CameraReader(camera_id=CAMERA_ID)

    # 2. 用ThreadedStream包装它，并启动读取线程
    threaded_reader = ThreadedStream(reader, queue_size=QUEUE_SIZE).start()

    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    print("初始化完成，读取线程已启动，开始检测循环...")
    print("按 'q' 键退出。")

except IOError as e:
    print(f"错误：无法打开摄像头。")
    print(f"详细信息: {e}")
    sys.exit(1)
except Exception as e:
    print(f"初始化时发生未知错误: {e}")
    sys.exit(1)

# ==========================================================
# 4. 主检测循环 (消费者)
# ==========================================================
try:
    # 循环条件：只要读取线程还在运行，或者队列中还有帧
    while threaded_reader.more():

        # 1. 从缓冲区获取最新的一帧 (此操作会阻塞，直到有新帧)
        frame = threaded_reader.read()

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
    # 5. 清理资源 (非常重要)
    print("正在停止读取线程并关闭窗口...")
    threaded_reader.stop()  # 向读取线程发送停止信号
    cv2.destroyAllWindows()
    print("程序退出。")