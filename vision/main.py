"""
视觉引导系统主程序。

管线：Camera -> YOLO(NPU) -> 缩放/灰度/二值化 -> 轮廓扫线 -> 路口判断 -> 串口发送 -> 显示
"""

from maix import camera, display, app, time, image
from yolo_detect import YoloDetector
from center_contour_scan import CenterContourScanner, draw_scan_overlay
from junction_detector import (
    JunctionDetector,
    draw_junction_guides,
    draw_junction_overlay,
)
import cv2
import serial

# ==================== 常用参数 ====================

MODEL_PATH = "/root/models/signpost_2_neck_int8.mud"
YOLO_INTERVAL = 5  # YOLO 推理间隔帧数
YOLO_CONF = 0.75
YOLO_IOU = 0.45
YOLO_DEBOUNCE = 3  # 连续识别 N 帧才确认（~45fps，每5帧推理，N=3≈0.33s）

LANE_SCALE = 0.25  # 图像缩放比例，320x256 -> 80x64
BIN_FIXED_TH = 115  # 二值化阈值，>=115 为白色赛道

JUNCTION_Y1 = 30  # 路口检测上边界：新路口必须在此行以上出现
JUNCTION_Y2 = 50  # 路口检测下边界：红线到达此行触发转弯

UART_PORT = "/dev/ttyS0"
UART_BAUD = 115200


# ==================== 扫线参数 ====================

ROW_STEP = 3  # 扫描行间距，越小越密
BLACK_RUN = 3  # 边界外侧连续黑像素数，用于过滤噪点
WHITE_TH = 200  # 白色像素阈值
BLACK_TH = 50  # 黑色像素阈值
MIN_WIDTH = 20  # 有效赛道最小宽度（像素）
CENTER_SEARCH_RADIUS = 30  # 中心点搜索半径
SCAN_OFFSET = 2  # 相邻行边界起始偏移


# ==================== 路口检测参数 ====================

JUNCTION_MIN_GAP_ROWS = 3  # 触发缺口的最小连续行数
JUNCTION_CONFIRM_FRAMES = 2  # 首次路口确认帧数
JUNCTION_NEW_CONFIRM_FRAMES = 2  # 搜索下一路口的确认帧数
JUNCTION_PROBE_WIDTH = 4  # 边界搜索范围（像素）
JUNCTION_WHITE_RUN = 3  # 判断缺口的连续白色像素数
JUNCTION_HOLD_FRAMES = 1  # 红线允许丢失的最大帧数
JUNCTION_TRACK_MAX_DY = 8  # 相邻帧红线最大位移


# ==================== 调试开关 ====================

SHOW_YOLO = True  # True：显示原图+YOLO框；False：显示二值赛道图
SHOW_FPS = True
SHOW_BOXES = True  # 绘制 YOLO 检测框，仅 SHOW_YOLO=True 时生效
PRINT_DETECT = True  # YOLO 检测到目标时打印终端日志
PRINT_JUNCTION = True  # 路口状态变化时打印终端日志
ENABLE_LANE_PIPELINE = True
ENABLE_CONTOUR_SCAN = True
ENABLE_JUNCTION_DETECT = True


# ==================== YOLO 标签映射 ====================

# W=直走, R=右转, L=左转, Y=绿灯
# direction: 0=直, 1=右, 2=左, 3=无
# green:     0=无, 1=绿灯
_DIRECTION_MAP = {"W": 0, "R": 1, "L": 2}
_GREEN_LABEL = "Y"


# ==================== 初始化 ====================

yolo = YoloDetector(MODEL_PATH, YOLO_INTERVAL, YOLO_CONF, YOLO_IOU, debounce=YOLO_DEBOUNCE)
yolo.init()

scanner = CenterContourScanner(
    row_step=ROW_STEP,
    black_run=BLACK_RUN,
    white_th=WHITE_TH,
    black_th=BLACK_TH,
    min_width=MIN_WIDTH,
    center_search_radius=CENTER_SEARCH_RADIUS,
    scan_offset=SCAN_OFFSET,
)

junction_detector = JunctionDetector(
    min_gap_rows=JUNCTION_MIN_GAP_ROWS,
    confirm_frames=JUNCTION_CONFIRM_FRAMES,
    new_confirm_frames=JUNCTION_NEW_CONFIRM_FRAMES,
    y1=JUNCTION_Y1,
    y2=JUNCTION_Y2,
    probe_width=JUNCTION_PROBE_WIDTH,
    black_run=BLACK_RUN,
    white_run=JUNCTION_WHITE_RUN,
    white_th=WHITE_TH,
    black_th=BLACK_TH,
    min_lane_width=MIN_WIDTH,
    hold_frames=JUNCTION_HOLD_FRAMES,
    track_max_dy=JUNCTION_TRACK_MAX_DY,
)


def _init_camera(yolo):
    # 优先使用模型输入尺寸，无模型时回退到 320x256
    if yolo.available:
        w, h, fmt = yolo.input_width, yolo.input_height, yolo.input_format
        print("[INFO] Camera: %dx%d (model input)" % (w, h))
        return camera.Camera(w, h, fmt)
    print("[INFO] Camera: 320x256 (fallback)")
    return camera.Camera(320, 256, image.Format.FMT_RGB888)


def _init_uart():
    try:
        port = serial.Serial(UART_PORT, UART_BAUD, timeout=0)
        print("[INFO] UART: %s @ %d" % (UART_PORT, UART_BAUD))
        return port
    except Exception as e:
        print("[WARN] UART open failed: %s" % e)
        return None


cam = _init_camera(yolo)
disp = display.Display()
uart = _init_uart()

# 串口发送状态，持久化到 TURN_NOW 结束后清零
_uart_junction_active = False


# ==================== 图像预处理 ====================


def _preprocess(img, scale):
    # 缩放 -> 灰度 -> numpy
    lw = int(img.width() * scale)
    lh = int(img.height() * scale)
    small = img.resize(lw, lh)
    gray = small.to_format(image.Format.FMT_GRAYSCALE)
    return image.image2cv(gray, False, False)


def _binarize(gray_np, threshold):
    _, bin_np = cv2.threshold(gray_np, threshold, 255, cv2.THRESH_BINARY)
    return bin_np


# ==================== 串口发送 ====================


def _uart_send(uart, is_junction, direction, green, stop, deviation):
    if uart is None:
        return
    raw = bytes([is_junction, direction, green, stop, deviation & 0xFF])
    buf = bytearray([0xFF])
    for b in raw:
        if b == 0xFF:
            buf.extend(b"\x7D\x5F")
        elif b == 0xFE:
            buf.extend(b"\x7D\x5E")
        elif b == 0x7D:
            buf.extend(b"\x7D\x5D")
        else:
            buf.append(b)
    buf.append(0xFE)
    try:
        uart.write(bytes(buf))
    except Exception:
        pass


def _compute_deviation(scan_result, W):
    """取最下方有效行的中心点偏移量（正值=偏右，负值=偏左）。"""
    if scan_result is None:
        return 0
    _, _, x_center, valid = scan_result
    cx = W // 2
    # 从最下行向上找第一个有效行
    for y in range(len(valid) - 1, -1, -1):
        if valid[y] and x_center[y] >= 0:
            return x_center[y] - cx
    return 0


def _build_uart_payload(junction_detector, should_turn, scan_result, W):
    """根据当前帧状态构造串口数据字段。"""
    global _uart_junction_active

    # is_junction：TURN_NOW 开始持续为 1，回到 NORMAL 后清零
    if should_turn or junction_detector.state == junction_detector.TURN_NOW:
        _uart_junction_active = True
    if junction_detector.state == junction_detector.NORMAL:
        _uart_junction_active = False

    is_junction = 1 if _uart_junction_active else 0

    # direction / green / stop 直接读 YOLO 缓存字段
    deviation = _compute_deviation(scan_result, W)
    return is_junction, yolo.direction, yolo.green, yolo.stop, deviation


# ==================== 日志 ====================


def _log_detect(frame, objs):
    if not PRINT_DETECT or not objs:
        return
    parts = []
    for obj in objs:
        cid = obj.class_id
        label = yolo.labels[cid] if cid < len(yolo.labels) else str(cid)
        parts.append(
            "%s %.2f box=(%d,%d,%d,%d)" % (label, obj.score, obj.x, obj.y, obj.w, obj.h)
        )
    print("[frame %d] DETECT: %s" % (frame, "; ".join(parts)))


def _log_junction(frame, detector, prev_status, should_turn):
    # 仅在状态变化时打印
    cur = (detector.state, detector.search_mode)
    if not PRINT_JUNCTION or cur == prev_status:
        return cur

    if detector.junction_y >= 0:
        pos = "y=%d x=(%d,%d)" % (
            detector.junction_y,
            detector.junction_left,
            detector.junction_right,
        )
    else:
        pos = "line=none"

    action = " TURN" if should_turn else ""
    print(
        "[frame %d] JUNCTION: %s -> %s mode=%s %s%s"
        % (frame, prev_status[0], detector.state, detector.search_mode, pos, action)
    )
    return cur


# ==================== 显示 ====================


_DIR_LABEL = {0: "W", 1: "R", 2: "L", 3: "-"}

def _draw_hud(img, fps, is_junction, deviation):
    """在图像上绘制 FPS、路口状态、方向、绿灯和偏差。字号 0.75，行距 15px。"""
    y = 2
    if SHOW_FPS:
        img.draw_string(2, y, "%.1f" % fps, image.COLOR_BLUE, 0.75)
        y += 15

    # 路口：J:1 红色 / J:0 绿色
    j_color = image.COLOR_RED if is_junction else image.COLOR_GREEN
    img.draw_string(2, y, "J:%d" % is_junction, j_color, 0.75)
    y += 15

    # 方向：W / L / R，红色
    img.draw_string(2, y, _DIR_LABEL.get(yolo.direction, "W"), image.COLOR_RED, 0.75)
    y += 15

    # 绿灯：有绿灯时显示 Y，绿色
    if yolo.green:
        img.draw_string(2, y, "Y", image.COLOR_GREEN, 0.75)
        y += 15

    # 停车：检测到 STOP 时显示 S，红色
    if yolo.stop:
        img.draw_string(2, y, "S", image.COLOR_RED, 0.75)
        y += 15

    # 偏差
    img.draw_string(2, y, "D:%+d" % deviation, image.COLOR_BLUE, 0.75)


def _show_binary(bin_np, fps, scan_result, jdet, is_junction, deviation):
    overlay = cv2.cvtColor(bin_np, cv2.COLOR_GRAY2RGB)

    if scan_result is not None:
        x_left, x_right, x_center, valid = scan_result
        draw_scan_overlay(overlay, x_left, x_right, x_center, valid)

    if jdet is not None:
        draw_junction_guides(overlay, jdet.y1, jdet.y2)
        draw_junction_overlay(overlay, jdet)

    maix_img = image.cv2image(overlay, False, True)
    if not maix_img:
        return

    _draw_hud(maix_img, fps, is_junction, deviation)
    disp.show(maix_img)


def _show_yolo(img, fps, objs, is_junction, deviation):
    _draw_hud(img, fps, is_junction, deviation)

    if SHOW_BOXES and objs:
        yolo.draw_debug(img, objs)

    # 退出按钮
    img.draw_rect(2, img.height() - 26, 68, 24, image.COLOR_RED, -1)
    img.draw_string(10, img.height() - 24, "EXIT", image.COLOR_WHITE, 1.0)

    disp.show(img)


# ==================== 主循环 ====================

print("[INFO] model: %s" % MODEL_PATH)

frame = 0
scan_result = None
bin_np = None
should_turn = False
last_junc_status = (junction_detector.state, junction_detector.search_mode)

while not app.need_exit():
    fps = time.fps()
    img = cam.read()

    # --- YOLO 推理 ---
    objs = yolo.detect(img, frame)
    _log_detect(frame, objs)

    # --- 赛道管线 ---
    if ENABLE_LANE_PIPELINE:
        gray_np = _preprocess(img, LANE_SCALE)
        bin_np = _binarize(gray_np, BIN_FIXED_TH)

        if frame == 0:
            print("[INFO] lane binary: %dx%d" % (bin_np.shape[1], bin_np.shape[0]))

        scan_result = scanner.apply(bin_np) if ENABLE_CONTOUR_SCAN else None

        if ENABLE_JUNCTION_DETECT and scan_result is not None:
            _, should_turn = junction_detector.update(bin_np, scan_result)
            last_junc_status = _log_junction(
                frame, junction_detector, last_junc_status, should_turn
            )

    # --- 串口发送 ---
    is_junc, direction, green, stop, deviation = _build_uart_payload(
        junction_detector, should_turn, scan_result, bin_np.shape[1] if bin_np is not None else 80)
    _uart_send(uart, is_junc, direction, green, stop, deviation)

    # --- 显示 ---
    if SHOW_YOLO or not ENABLE_LANE_PIPELINE:
        _show_yolo(img, fps, objs, is_junc, deviation)
    else:
        jdet = junction_detector if ENABLE_JUNCTION_DETECT else None
        _show_binary(bin_np, fps, scan_result, jdet, is_junc, deviation)

    frame += 1
