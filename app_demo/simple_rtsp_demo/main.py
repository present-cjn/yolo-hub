# app_demo/simple_rtsp_demo/main.py
import cv2
import sys
import os

# --- 1. 导入 core_lib ---
# 将项目根目录 (YoloHub) 添加到Python路径
# (.. -> app_demo, .. -> YoloHub)
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))
try:
    from core_lib.detector import Detector
    from core_lib.img_reader.rtsp_reader import RTSPReader
except ImportError:
    print("错误：无法从 core_lib 导入模块。")
    print("请确保 core_lib 文件夹在 YoloHub 根目录下。")
    sys.exit(1)

# ==========================================================
# 2. 配置参数 (直接写在脚本里)
# ==========================================================
# 模型路径 (位于上一级 app_demo 目录中)
MODEL_PATH = "../yolov8n.pt"

# TODO: ！！请将这里替换为您自己的RTSP流地址！！
RTSP_URL = "rtsp://YOUR_RTSP_STREAM_URL_HERE"

WINDOW_NAME = "YoloHub Simple RTSP Demo"

# ==========================================================
# 3. 初始化
# ==========================================================
try:
    print("正在初始化YOLO检测器...")
    detector = Detector(model_path=MODEL_PATH)

    print(f"正在连接RTSP流: {RTSP_URL}...")
    reader = RTSPReader(rtsp_url=RTSP_URL)  # <--- 初始化RTSPReader

    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    print("初始化完成，开始检测循环...")
    print("按 'q' 键退出。")

except IOError as e:
    print(f"错误：无法打开RTSP流。请检查URL是否正确以及网络是否可达。")
    print(f"详细信息: {e}")
    sys.exit(1)
except Exception as e:
    print(f"初始化时发生未知错误: {e}")
    sys.exit(1)

# ==========================================================
# 4. 主检测循环 (单线程)
# ==========================================================
try:
    # RTSPReader 实现了迭代器协议，可以直接在 for 循环中使用
    for frame in reader:

        # 1. 检测
        results = detector.detect(frame, verbose=False)

        # 2. 获取带标注的图像
        annotated_frame = results[0].plot()

        # 3. 显示结果
        cv2.imshow(WINDOW_NAME, annotated_frame)

        # 检查退出键
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("检测到 'q' 键，正在退出...")
            break

except KeyboardInterrupt:
    print("\n检测被用户中断 (Ctrl+C)。")

finally:
    # 5. 清理资源
    print("正在释放RTSP流并关闭窗口...")
    reader.release()
    cv2.destroyAllWindows()
    print("程序退出。")