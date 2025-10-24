# YoloHub/core_lib/publisher/http_publisher.py (升级版)
import requests
import cv2
import time


class HttpPublisher:
    def __init__(self, server_url: str, device_id: str):
        """
        初始化HTTP发布器。
        :param server_url: 基础服务器URL, e.g., "http://192.168.1.100:5000"
        :param device_id: 此设备的唯一ID, e.g., "orin_nx_01"
        """
        # 动态构建唯一的上传路径
        self.endpoint_url = f"{server_url}/upload/{device_id}"
        self.session = requests.Session()
        print(f"HTTP发布器已初始化，将上报至: {self.endpoint_url}")

    def publish(self, frame, quality=90):
        try:
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), quality]
            _, img_encoded = cv2.imencode('.jpg', frame, encode_param)
            files = {'image': ('frame.jpg', img_encoded.tobytes(), 'image/jpeg')}

            self.session.post(self.endpoint_url, files=files, timeout=0.5)

        except requests.exceptions.RequestException as e:
            print(f"警告：无法发送数据到服务器: {e}")
        except Exception as e:
            print(f"警告：发布时发生未知错误: {e}")