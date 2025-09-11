# core_lib/handlers/robust_trigger_handler.py
from collections import deque
from datetime import datetime


class RobustTriggerHandler:
    def __init__(self, target_class_name: str, window_size: int, count_threshold: int, area_threshold: float,
                 action_handlers: list):
        # 判断条件
        self.target_class_name = target_class_name
        self.window_size = window_size
        self.count_threshold = count_threshold
        self.area_threshold = area_threshold

        # 内部状态
        self.detection_history = deque(maxlen=self.window_size)
        self.trigger_state = False
        self.target_class_id = None

        # 触发后要执行的动作处理器列表
        self.action_handlers = action_handlers

    def handle(self, frame, results):
        # 首次运行时，从模型动态获取class_id
        if self.target_class_id is None:
            try:
                model_names = results[0].names
                self.target_class_id = list(model_names.keys())[
                    list(model_names.values()).index(self.target_class_name)]
                print(f"'{self.target_class_name}' 对应的类别ID为 {self.target_class_id}。")
            except (ValueError, IndexError):
                print(f"错误：在模型类别中未找到 '{self.target_class_name}'。")
                # 阻止后续处理
                return

        # 1. 数据处理：分析当前帧
        person_detected_in_frame = False
        max_area_in_frame = 0.0

        for box in results[0].boxes:
            if box.cls == self.target_class_id:
                person_detected_in_frame = True
                x1, y1, x2, y2 = box.xyxy[0]
                area = (x2 - x1) * (y2 - y1)
                if area > max_area_in_frame:
                    max_area_in_frame = area

        self.detection_history.append((person_detected_in_frame, max_area_in_frame))

        # 2. 决策逻辑：仅当历史队列填满时判断
        if len(self.detection_history) < self.window_size:
            return

        detection_count = sum(1 for detected, area in self.detection_history if detected)
        max_area_in_window = max(
            area for detected, area in self.detection_history if detected) if detection_count > 0 else 0

        conditions_met = (detection_count >= self.count_threshold) and (max_area_in_window >= self.area_threshold)

        # 3. 触发动作
        if conditions_met:
            if not self.trigger_state:
                print(
                    f"[{datetime.now().strftime('%H:%M:%S')}] 条件满足: {detection_count}/{self.window_size}帧, 最大面积{int(max_area_in_window)} >= {self.area_threshold}. 触发动作!")
                self.trigger_state = True

                # 创建上下文信息，传递给动作处理器
                context = {
                    "detection_count": detection_count,
                    "max_area_in_window": max_area_in_window
                }

                # 调用所有注入的动作处理器
                for handler in self.action_handlers:
                    handler.handle(frame, results, context)
        else:
            if self.trigger_state:
                print(f"[{datetime.now().strftime('%H:%M:%S')}] 条件不再满足。重置状态。")
                self.trigger_state = False