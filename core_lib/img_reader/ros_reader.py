# core_lib/inputs/ros_image_reader.py

import threading


class ROSImageReader:
    """
    一个包装器，用于订阅ROS图像话题并在回调中存储最新的帧。
    它不是一个迭代器，而是提供一个 .read() 方法来获取最新帧。
    """

    def __init__(self, topic_name: str, node_name: str = 'yolohub_ros_reader'):
        self.topic_name = topic_name
        self.node_name = node_name
        self.latest_frame = None
        self.lock = threading.Lock()

        try:
            import rospy
            from sensor_msgs.msg import Image
            from cv_bridge import CvBridge
        except ImportError:
            print("错误：ROS依赖未满足。")
            raise ImportError("请确保已安装 rospy, sensor_msgs, 和 cv_bridge (通常通过 apt 安装)。")

        self.rospy = rospy
        self.bridge = CvBridge()

        # 初始化节点（如果尚未初始化）
        if not self.rospy.core.is_initialized():
            self.rospy.init_node(self.node_name, anonymous=True)
            print(f"ROS 节点 '{self.node_name}' 已初始化。")

        # 订阅话题
        self.subscriber = self.rospy.Subscriber(self.topic_name, Image, self._callback)
        print(f"已订阅 ROS 图像话题: {self.topic_name}")

    def _callback(self, msg):
        """ROS回调函数，在独立的线程中运行。"""
        try:
            # 将ROS Image消息转换为OpenCV图像 (bgr8)
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            with self.lock:
                self.latest_frame = cv_image
        except Exception as e:
            self.rospy.logerr(f"CVBridge 转换错误: {e}")

    def read(self):
        """
        以线程安全的方式获取最新的帧。
        这是一个非阻塞方法。
        :return: 最新的OpenCV帧 (numpy array) 或 None
        """
        with self.lock:
            if self.latest_frame is not None:
                return self.latest_frame.copy()
        return None

    def is_ready(self):
        """检查是否已收到至少一帧。"""
        with self.lock:
            return self.latest_frame is not None

    def release(self):
        """停止订阅。"""
        if hasattr(self, 'subscriber'):
            self.subscriber.unregister()
            print(f"已取消订阅 {self.topic_name}")