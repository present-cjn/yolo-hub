# core_lib/inputs/camera_reader.py
import cv2

class CameraReader:
    def __init__(self, camera_id: int = 0):
        self.camera_id = camera_id
        self.cap = cv2.VideoCapture(self.camera_id)
        if not self.cap.isOpened():
            raise IOError(f"错误：无法打开摄像头 {self.camera_id}")
        print(f"摄像头 {self.camera_id} 打开成功。")

    def __iter__(self):
        return self

    def __next__(self):
        ret, frame = self.cap.read()
        if not ret:
            print("警告：无法从摄像头读取到帧。")
            raise StopIteration
        return frame

    def release(self):
        self.cap.release()
        print("摄像头资源已释放。")