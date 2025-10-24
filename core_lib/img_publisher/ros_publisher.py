# core_lib/img_publisher/ros_publisher.py

class ROSImagePublisher:
    def __init__(self, topic_name: str, node_name: str = 'yolohub_image_publisher'):
        """
        初始化ROS图像发布器。
        """
        self.topic_name = topic_name
        self.node_name = node_name

        try:
            import rospy
            from sensor_msgs.msg import Image
            from cv_bridge import CvBridge
        except ImportError:
            raise ImportError("ROS依赖未满足 (rospy, sensor_msgs, cv_bridge)。")

        self.bridge = CvBridge()
        self.rospy = rospy

        # 初始化节点（如果尚未初始化）
        if not self.rospy.core.is_initialized():
            self.rospy.init_node(self.node_name, anonymous=True)
            print(f"ROS 节点 '{self.node_name}' 已初始化。")

        # 创建发布者，队列大小为1，因为对于视频流，我们只关心最新的一帧
        self.publisher = self.rospy.Publisher(self.topic_name, Image, queue_size=1)
        print(f"ROS 图像发布器初始化成功。将发布到: {self.topic_name}")

    def publish(self, frame, frame_id="annotated_image"):
        """
        将OpenCV图像帧发布到ROS话题。
        :param frame: OpenCV 图像 (numpy array, bgr8)
        :param frame_id: ROS消息头中的 frame_id
        """
        if self.rospy.is_shutdown():
            return

        try:
            # 将OpenCV图像 (bgr8) 转换为ROS Image消息
            ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            ros_image.header.stamp = self.rospy.Time.now()
            ros_image.header.frame_id = frame_id

            # 发布消息
            self.publisher.publish(ros_image)
        except Exception as e:
            self.rospy.logerr(f"CVBridge 转换或发布错误: {e}")