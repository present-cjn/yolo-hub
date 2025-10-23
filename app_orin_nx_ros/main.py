# app_orin_nx_ros/main.py
import sys
import os
import yaml

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

# 导入所有需要的积木
from core_lib.detector import Detector
from core_lib.img_reader.camera_reader import CameraReader
from core_lib.img_reader.rtsp_reader import RTSPReader
from core_lib.threaded_stream import ThreadedStream
from core_lib.handlers.robust_trigger_handler import RobustTriggerHandler
from core_lib.handlers.file_saver_handler import FileSaverHandler
from core_lib.handlers.ros_bool_publisher_handler import ROSBoolPublisherHandler

# 将处理器名称映射到类
HANDLER_MAP = {
    "FileSaver": FileSaverHandler,
    "ROSBoolPublisher": ROSBoolPublisherHandler,
    "RobustTriggerHandler": RobustTriggerHandler
}


def main():
    with open('config.yaml', 'r') as f:
        config = yaml.safe_load(f)

    # --- 1. 初始化检测器和视频流 ---
    detector = Detector(**config['detector_params'])

    input_cfg = config['input_source']
    if input_cfg['type'] == 'camera':
        original_reader = CameraReader(**input_cfg['camera_params'])
    elif input_cfg['type'] == 'rtsp':
        original_reader = RTSPReader(**input_cfg['rtsp_params'])
    else:
        raise ValueError(f"不支持的输入类型: {input_cfg['type']}")

    threaded_reader = ThreadedStream(original_reader, **config.get('threaded_stream_params', {})).start()

    # --- 2. 组装处理器 (核心步骤) ---
    main_handler_cfg = config['main_handler']

    # a. 先创建所有“动作”处理器
    action_handlers = []
    for action_cfg in main_handler_cfg['params']['action_handlers']:
        ActionHandlerClass = HANDLER_MAP[action_cfg['type']]
        action_handlers.append(ActionHandlerClass(**action_cfg['params']))

    # b. 创建主处理器，并将动作处理器列表作为参数注入进去
    RobustHandlerClass = HANDLER_MAP[main_handler_cfg['type']]
    # 提取RobustTriggerHandler自身的参数，并添加action_handlers
    robust_handler_params = main_handler_cfg['params'].copy()
    robust_handler_params['action_handlers'] = action_handlers

    main_handler = RobustHandlerClass(**robust_handler_params)

    print("鲁棒性检测应用启动...")
    try:
        while threaded_reader.more():
            frame = threaded_reader.read()
            # 推理时关闭详细输出，保持控制台干净
            results = detector.detect(frame, verbose=False)
            # 只需要调用主处理器即可
            main_handler.handle(frame, results)
    except KeyboardInterrupt:
        print("\n应用终止。")
    finally:
        threaded_reader.stop()
        print("所有资源已清理。")


if __name__ == '__main__':
    main()