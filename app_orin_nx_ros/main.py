# app_orin_nx_ros/main.py
import sys
import os
import yaml

# 将项目根目录添加到Python路径，以便能找到core_lib
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from core_lib.detector import Detector
from core_lib.inputs.rtsp_reader import RTSPReader
from core_lib.threaded_stream import ThreadedStream
from core_lib.handlers.file_saver_handler import FileSaverHandler
from core_lib.handlers.ros_publisher_handler import ROSPublisherHandler

# 将处理器名称映射到类
HANDLER_MAP = {
    "FileSaver": FileSaverHandler,
    "ROSPublisher": ROSPublisherHandler,
}


def main():
    # 加载此应用的专属配置
    with open('config.yaml', 'r') as f:
        config = yaml.safe_load(f)

    # --- 组装流水线 ---
    detector = Detector(**config['detector_params'])
    original_reader = RTSPReader(**config['reader_params'])
    threaded_reader = ThreadedStream(original_reader, **config.get('threaded_stream_params', {})).start()

    handlers = []
    for handler_config in config.get('handlers', []):
        handler_name = handler_config.get('name')
        if handler_name in HANDLER_MAP:
            HandlerClass = HANDLER_MAP[handler_name]
            handlers.append(HandlerClass(**handler_config.get('params', {})))
        else:
            print(f"警告：未知的处理器名称 '{handler_name}'，已跳过。")

    if not handlers:
        print("警告：没有配置任何处理器，程序将只进行检测。")

    print("Orin NX 应用启动：多线程读取RTSP，检测并处理...")
    try:
        while threaded_reader.more():
            frame = threaded_reader.read()
            results = detector.detect(frame)

            # 将结果交给所有处理器处理
            for handler in handlers:
                handler.handle(frame, results)

    except KeyboardInterrupt:
        print("\n应用终止。")
    finally:
        threaded_reader.stop()
        print("所有资源已清理。")


if __name__ == '__main__':
    main()