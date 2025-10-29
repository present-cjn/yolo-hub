import cv2
import sys
import os
import glob  # 用于查找文件

# --- 1. 导入 core_lib ---
# (.. -> app_demo, .. -> YoloHub)
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))
try:
    from core_lib.detector import Detector
except ImportError:
    print("错误：无法从 core_lib 导入模块。")
    sys.exit(1)

# ==========================================================
# 2. 配置参数
# ==========================================================
# TODO: !! 将这里修改为您要处理的媒体的 "绝对路径" !!
# 示例 1 (单张图片): r"C:\Users\YourUser\Pictures\test.jpg"
# 示例 2 (图片文件夹): r"C:\Users\YourUser\Pictures\My_Dataset"
# 示例 3 (视频文件): r"C:\Users\YourUser\Videos\my_test_video.mp4"
# (Linux/Mac 路径: "/home/user/my_video.mp4")
INPUT_PATH = "/media/hzbz/ZNJS/cjn/jtjj_1028/output_data_unc001/images/original/unc001_1761638155962.jpg"

# 支持的图片和视频格式 (用于文件夹扫描)
IMAGE_EXTENSIONS = ["*.jpg", "*.jpeg", "*.png", "*.bmp", "*.webp"]
VIDEO_EXTENSIONS = ["*.mp4", "*.avi", "*.mkv", "*.mov"]

MODEL_PATH = "../best1020.pt"
CONF_THRESHOLD = 0.25
IOU_THRESHOLD = 0.7
WINDOW_NAME = "YoloHub Offline Media Processor"


# ==========================================================
# 3. 辅助函数 (用于处理不同的循环)
# ==========================================================

def process_media(detector, media_files, is_video=False):
    """
    统一处理循环：可以是图片文件列表，也可以是VideoCap对象。
    """
    for media_source in media_files:
        try:
            if is_video:
                # media_source 是 VideoCapture 对象
                ret, frame = media_source.read()
                if not ret:
                    print("视频播放完毕。")
                    break
                current_file_name = "Video Frame"
            else:
                # media_source 是图片文件路径
                frame = cv2.imread(media_source)
                if frame is None:
                    print(f"警告：无法读取图片 {media_source}，已跳过。")
                    continue
                current_file_name = media_source

            # --- 核心检测逻辑 ---
            results = detector.detect(frame, verbose=False, conf=CONF_THRESHOLD, iou=IOU_THRESHOLD, agnostic_nms=True)
            num_detected_boxes = len(results[0].boxes)
            print(f"一共检测到 {num_detected_boxes} 个框。")
            print(results[0].boxes)
            annotated_frame = results[0].plot()
            # --- ---------------- ---

            # ----------------------------------------
            # 将检测到的数量打印到图片的左上角
            # ----------------------------------------

            # 文本内容
            text_to_display = f"Detected: {num_detected_boxes}"

            # 字体设置
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 1  # 字体大小
            font_thickness = 2  # 字体粗细
            text_color = (0, 0, 255)  # 文本颜色 (B, G, R)，这里是红色
            text_position = (10, 30)  # 文本起始位置 (x, y)，左上角

            # 将文本绘制到图片上
            cv2.putText(annotated_frame,
                        text_to_display,
                        text_position,
                        font,
                        font_scale,
                        text_color,
                        font_thickness,
                        cv2.LINE_AA)  # 抗锯齿

            # ----------------------------------------

            print(f"正在显示: {current_file_name}")
            cv2.imshow(WINDOW_NAME, annotated_frame)

            # 根据是视频还是图片，决定等待时间
            wait_key_time = 1 if is_video else 0  # 视频: 等1ms; 图片: 无限等
            key = cv2.waitKey(wait_key_time)

            if key == ord('q'):
                print("检测到 'q' 键，正在退出...")
                return "quit"  # 返回退出信号
            elif key == ord('n') and not is_video:
                continue  # 图片模式：按 'n' 切换下一张

        except KeyboardInterrupt:
            return "quit"  # 捕获 Ctrl+C
    return "done"


# ==========================================================
# 4. 主程序：初始化和路径判断
# ==========================================================
def main():
    print("正在初始化YOLO检测器...")
    detector = Detector(model_path=MODEL_PATH)
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)

    media_list = []
    is_video = False

    if not os.path.exists(INPUT_PATH):
        print(f"错误：路径不存在: {INPUT_PATH}")
        return

    # --- 核心路径判断逻辑 ---
    if os.path.isdir(INPUT_PATH):
        # 1. 如果是文件夹 -> 扫描所有图片
        print(f"检测到文件夹，正在扫描图片... {INPUT_PATH}")
        for ext in IMAGE_EXTENSIONS:
            media_list.extend(glob.glob(os.path.join(INPUT_PATH, ext)))
        if not media_list:
            print(f"错误：在文件夹 {INPUT_PATH} 中未找到支持的图片文件。")
            return
        print(f"找到 {len(media_list)} 张图片。按 'n' 键查看下一张，按 'q' 键退出。")

    elif os.path.isfile(INPUT_PATH):
        _, ext = os.path.splitext(INPUT_PATH)
        ext = ext.lower()

        if ext in [e.replace("*", "") for e in IMAGE_EXTENSIONS]:
            # 2. 如果是单张图片
            print(f"检测到单张图片: {INPUT_PATH}")
            media_list.append(INPUT_PATH)
            print("按 'n' 或 'q' 键退出。")

        elif ext in [e.replace("*", "") for e in VIDEO_EXTENSIONS]:
            # 3. 如果是视频文件
            print(f"检测到视频文件: {INPUT_PATH}")
            cap = cv2.VideoCapture(INPUT_PATH)
            if not cap.isOpened():
                print(f"错误：无法打开视频文件 {INPUT_PATH}")
                return
            media_list = [cap]  # 将cap对象作为"源"
            is_video = True
            print("视频播放中... 按 'q' 键退出。")

        else:
            print(f"错误：不支持的文件类型: {ext}")
            return

    # --- 执行处理 ---
    try:
        process_media(detector, media_list, is_video)
    finally:
        # --- 清理资源 ---
        if is_video and media_list:
            media_list[0].release()  # 释放 VideoCapture
        cv2.destroyAllWindows()
        print("程序退出。")


if __name__ == "__main__":
    main()