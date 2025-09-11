# core_lib/handlers/file_saver_handler.py
import cv2
import os
from datetime import datetime


class FileSaverHandler:
    def __init__(self, save_path: str, prefix='detection'):
        self.save_path = save_path
        self.prefix = prefix
        if not os.path.exists(self.save_path):
            os.makedirs(self.save_path)
            print(f"创建保存目录: {self.save_path}")

    def handle(self, frame, results):
        # 仅在检测到目标时保存
        if len(results[0].boxes) > 0:
            # 使用ultralytics自带的绘图功能
            annotated_frame = results[0].plot()

            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            filename = f"{self.prefix}_{timestamp}.jpg"
            filepath = os.path.join(self.save_path, filename)

            cv2.imwrite(filepath, annotated_frame)
            # print(f"检测结果已保存至: {filepath}")