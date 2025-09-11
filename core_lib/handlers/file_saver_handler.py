# core_lib/handlers/file_saver_handler.py
import cv2
import os
from datetime import datetime


class FileSaverHandler:
    def __init__(self, base_path: str, filename_format: str = "{time}_trigger.jpg"):
        self.base_path = base_path
        self.filename_format = filename_format

    def handle(self, frame, results, context=None):
        """
        保存带有标注的图像。
        新增 context 参数，可以从其他handler接收额外信息。
        """
        now = datetime.now()
        date_folder = now.strftime("%Y-%m-%d")
        save_directory = os.path.join(self.base_path, date_folder)
        os.makedirs(save_directory, exist_ok=True)

        # 格式化文件名
        time_str = now.strftime("%H%M%S_%f")[:-3]

        # 从上下文中获取额外信息来丰富文件名
        detection_count = context.get('detection_count', 0) if context else 0
        max_area = context.get('max_area_in_window', 0) if context else 0

        filename = self.filename_format.format(
            time=time_str,
            count=detection_count,
            area=int(max_area)
        )

        save_path = os.path.join(save_directory, filename)
        annotated_frame = results[0].plot()
        cv2.imwrite(save_path, annotated_frame)
        print(f"    -> 触发快照已保存至: {save_path}")