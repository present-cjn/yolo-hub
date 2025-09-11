# core_lib/detector.py
from ultralytics import YOLO

class Detector:
    def __init__(self, model_path: str):
        """
        初始化检测器，加载模型。
        :param model_path: .pt 模型文件的路径
        """
        self.model = YOLO(model_path)
        print(f"模型 '{model_path}' 加载成功。")

    def detect(self, frame, verbose=False):
        """
        对单帧图像进行检测。
        :param frame: 输入的图像 (numpy array)
        :param verbose: 是否打印详细检测信息
        :return: ultralytics的检测结果对象
        """
        return self.model(frame, verbose=verbose)