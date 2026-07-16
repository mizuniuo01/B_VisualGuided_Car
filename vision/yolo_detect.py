"""
yolo_detect.py — YOLO 推理封装

标志类别映射：
    W → direction=0（直走）
    R → direction=1（右转）
    L → direction=2（左转）
    Y → green=1（绿灯）
    STOP → stop=1（停车）
"""

from maix import nn

# ==================== 标志映射 ====================

# label → (direction, green)
LABEL_MAP = {
    "W": (0, 0),
    "R": (1, 0),
    "L": (2, 0),
    "Y": (0, 1),
}

STOP_LABEL = "STOP"


# ==================== 检测器 ====================


class YoloDetector:

    def __init__(self, model_path, interval=5, conf_th=0.5, iou_th=0.45, debounce=3):
        """
        model_path : .mud 模型文件路径
        interval   : 推理间隔帧数，其余帧返回缓存
        conf_th    : 置信度阈值
        iou_th     : IoU 阈值
        debounce   : 连续识别 N 次才确认，防抖
        """
        self.model_path = model_path
        self.interval = interval
        self.conf_th = conf_th
        self.iou_th = iou_th
        self.debounce = debounce

        self._detector = None
        self._available = False
        self.labels = []

        # 对外输出字段，direction 默认 3（无标志）
        self.direction = 3
        self.green = 0
        self.stop = 0

        # 防抖内部状态（direction/green 共用一个计数器，stop 独立）
        self._pending_direction = 3
        self._pending_green = 0
        self._debounce_count = 0

        self._pending_stop = 0
        self._stop_debounce_count = 0

    def init(self):
        """加载模型；失败时降级为空结果，不抛异常。"""
        try:
            self._detector = nn.YOLOv5(model=self.model_path)
            self._available = True
            self.labels = list(self._detector.labels)
            print("[INFO] YOLO loaded: %s" % self.model_path)
            print("[INFO] Labels: %s" % self.labels)
        except Exception as e:
            print("[WARN] YOLO load failed: %s" % e)

    # ---------- 属性 ----------

    @property
    def available(self):
        return self._available

    @property
    def input_width(self):
        return self._detector.input_width() if self._available else 320

    @property
    def input_height(self):
        return self._detector.input_height() if self._available else 240

    @property
    def input_format(self):
        return self._detector.input_format() if self._available else None

    # ---------- 推理 ----------

    def detect(self, img, frame_id):
        """
        按 interval 执行推理，其余帧返回缓存。

        返回 objs 列表（推理帧为真实结果，缓存帧为空列表）。
        direction / green 字段同步更新。
        """
        if not self._available:
            return []

        if frame_id % self.interval != 0:
            return []

        objs = self._detector.detect(img, conf_th=self.conf_th, iou_th=self.iou_th)

        # 提取本帧检测结果
        detected_direction = 3
        detected_green = 0
        detected_stop = 0

        for obj in objs:
            cid = obj.class_id
            label = self.labels[cid] if cid < len(self.labels) else None
            if label == STOP_LABEL:
                detected_stop = 1
            elif label in LABEL_MAP:
                d, g = LABEL_MAP[label]
                if g:
                    detected_green = 1
                else:
                    detected_direction = d

        # 防抖：同一结果连续 debounce 帧才对外更新
        if (detected_direction, detected_green) == (self._pending_direction, self._pending_green):
            self._debounce_count += 1
        else:
            self._pending_direction = detected_direction
            self._pending_green = detected_green
            self._debounce_count = 1

        if self._debounce_count >= self.debounce:
            self.direction = self._pending_direction
            self.green = self._pending_green

        # STOP 独立防抖
        if detected_stop == self._pending_stop:
            self._stop_debounce_count += 1
        else:
            self._pending_stop = detected_stop
            self._stop_debounce_count = 1

        if self._stop_debounce_count >= self.debounce:
            self.stop = self._pending_stop

        return objs

    def draw_debug(self, img, objs):
        """在图像上绘制检测框（调试用）。"""
        if not objs:
            return
        from maix import image

        for obj in objs:
            img.draw_rect(obj.x, obj.y, obj.w, obj.h, image.COLOR_RED, 1)
            label = (
                self.labels[obj.class_id]
                if obj.class_id < len(self.labels)
                else str(obj.class_id)
            )
            img.draw_string(
                obj.x, obj.y, "%s:%.2f" % (label, obj.score), image.COLOR_RED, 1.0
            )
