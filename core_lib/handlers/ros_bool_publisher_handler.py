# core_lib/handlers/ros_bool_publisher_handler.py
class ROSBoolPublisherHandler:
    def __init__(self, topic_name: str, node_name='yolohub_bool_publisher'):
        self.topic_name = topic_name
        self.node_name = node_name
        self.publisher = None
        self._initialize_ros()

    def _initialize_ros(self):
        try:
            import rospy
            from std_msgs.msg import Bool
            self.BoolMsg = Bool
        except ImportError:
            raise ImportError("无法导入 rospy 或 std_msgs.msg.Bool。")

        if not rospy.core.is_initialized():
            rospy.init_node(self.node_name, anonymous=True)

        self.publisher = rospy.Publisher(self.topic_name, self.BoolMsg, queue_size=10)
        print(f"ROS Bool 处理器初始化成功，将发布到话题: {self.topic_name}")

    def handle(self, frame, results, context=None):
        # 这个处理器只发布 True，并且由 RobustTriggerHandler 调用
        self.publisher.publish(True)
        print(f"    -> ROS 消息 'True' 已发布到 {self.topic_name}")