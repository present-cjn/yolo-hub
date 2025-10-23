# app_simple_demo/run_demo.py
import cv2
import sys
import os

# --- 1. 导入 core_lib ---
# 将项目根目录添加到Python路径
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
try:
    from core_lib.detector import Detector
    from core_lib.img_reader.camera_reader import CameraReader
    # 注意：这个简单demo不使用多线程
except ImportError:
    print("错误：无法从 core_lib 导入模块。")
    print("请确保 core_lib 文件夹在 YoloHub 根目录下。")
    sys.exit(1)

# ==========================================================
# 2. 配置参数 (直接写在脚本里)
# ==========================================================
# TODO: 请确保你有 'yolov8n.pt' 文件，或者改成你自己的模型路径
MODEL_PATH = "../yolov8n.pt"
CAMERA_ID = 0
WINDOW_NAME = "YoloHub Simple Demo"

# ==========================================================
# 3. 初始化
# ==========================================================
print("正在初始化YOLO检测器...")
detector = Detector(model_path=MODEL_PATH)

print(f"正在打开摄像头 {CAMERA_ID}...")
reader = CameraReader(camera_id=CAMERA_ID)

cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
print("初始化完成，开始检测循环...")
print("按 'q' 键退出。")

# ==========================================================
# 4. 主检测循环 (单线程)
# ==========================================================
try:
    # CameraReader 实现了迭代器协议，可以直接在 for 循环中使用
    for frame in reader:

        # 1. 检测
        # verbose=False 可以让控制台保持干净
        results = detector.detect(frame, verbose=False)

        # 2. 获取带标注的图像
        # results[0].plot() 会返回一个绘制了检测框的 frame
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
    print("正在释放摄像头并关闭窗口...")
    reader.release()
    cv2.destroyAllWindows()
    print("程序退出。")