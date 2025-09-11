# core_lib/inputs/rtsp_reader.py
import cv2


class RTSPReader:
    def __init__(self, rtsp_url: str):
        self.rtsp_url = rtsp_url
        # Orin NX上使用GStreamer后端通常性能更好
        self.cap = cv2.VideoCapture(self.rtsp_url, cv2.CAP_GSTREAMER)
        if not self.cap.isOpened():
            print(f"警告：无法使用GStreamer打开RTSP流，尝试默认后端...")
            self.cap = cv2.VideoCapture(self.rtsp_url)

        if not self.cap.isOpened():
            raise IOError(f"错误：无法打开RTSP流: {self.rtsp_url}")
        print(f"RTSP流 '{self.rtsp_url}' 连接成功。")

    def __iter__(self):
        return self

    def __next__(self):
        ret, frame = self.cap.read()
        if not ret:
            print("警告：无法从RTSP流读取到帧，可能已断开。")
            raise StopIteration
        return frame

    def release(self):
        self.cap.release()
        print("RTSP流资源已释放。")