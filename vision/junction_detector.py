"""junction_detector.py — 路口检测与状态跟踪。"""

# ====================  状态常量  ====================


class JunctionDetector:

    # 主状态
    NORMAL = "NORMAL"  # 正常行驶，搜索路口缺口
    APPROACH = "APPROACH"  # 已确认路口，跟踪红线下移
    TURN_NOW = "TURN_NOW"  # 红线越过 y2，触发转弯
    PASSING = "PASSING"  # 转弯后，等待路口离开视野

    # 搜索模式
    TRACK_CURRENT = "TRACK_CURRENT"  # 跟踪当前路口
    SEARCH_NEXT = "SEARCH_NEXT"  # 搜索下一个路口

    # ====================  初始化  ====================

    def __init__(
        self,
        min_gap_rows=2,
        confirm_frames=2,
        new_confirm_frames=2,
        y1=20,
        y2=42,
        probe_width=4,
        black_run=3,
        white_run=3,
        white_th=200,
        black_th=50,
        min_lane_width=20,
        hold_frames=1,
        track_max_dy=8,
    ):
        if min_gap_rows < 1:
            raise ValueError("min_gap_rows must be at least 1")
        if confirm_frames < 1 or new_confirm_frames < 1:
            raise ValueError("confirm frame counts must be at least 1")
        if y1 >= y2:
            raise ValueError("y1 must be less than y2")
        if probe_width < 1:
            raise ValueError("probe_width must be at least 1")

        # 参数
        self.min_gap_rows = min_gap_rows
        self.confirm_frames = confirm_frames
        self.new_confirm_frames = new_confirm_frames
        self.y1 = y1
        self.y2 = y2
        self.probe_width = probe_width
        self.black_run = black_run
        self.white_run = white_run
        self.white_th = white_th
        self.black_th = black_th
        self.min_lane_width = min_lane_width
        self.hold_frames = hold_frames
        self.track_max_dy = track_max_dy

        # 对外状态
        self.state = self.NORMAL
        self.search_mode = self.TRACK_CURRENT
        self.junction_y = -1
        self.junction_left = -1
        self.junction_right = -1

        # 内部跟踪
        self._candidate = None
        self._confirm_count = 0
        self._missing_count = 0
        self._new_candidate = None
        self._new_confirm_count = 0

    # ====================  主接口  ====================

    def update(self, binary_np, scan_result):
        """每帧调用，返回 (is_intersection, should_turn)。"""
        if self.state == self.NORMAL:
            candidate = self._find_gap(binary_np, scan_result, max_line_y=self.y1 - 1)
            return self._update_normal(candidate)

        if self.state == self.TURN_NOW:
            self.state = self.PASSING

        if self.state == self.APPROACH:
            candidate = self._find_gap(binary_np, scan_result)
            return self._update_approach(candidate)

        return self._update_passing(binary_np, scan_result)

    def reset(self):
        """重置跟踪状态。"""
        self._reset_current()

    # ====================  状态更新  ====================

    def _update_normal(self, candidate):
        if candidate is None:
            self._candidate = None
            self._confirm_count = 0
            return False, False

        if self._same_candidate(candidate, self._candidate):
            self._confirm_count += 1
        else:
            self._confirm_count = 1
        self._candidate = candidate

        if self._confirm_count < self.confirm_frames:
            return False, False

        # 确认路口，切换到 APPROACH
        self._publish(candidate)
        self.state = self.APPROACH
        self.search_mode = self.TRACK_CURRENT
        self._missing_count = 0
        return True, False

    def _update_approach(self, candidate):
        if candidate is None or not self._same_candidate(candidate, self._candidate):
            self._missing_count += 1
            if self._missing_count > self.hold_frames:
                self._reset_current()
                return False, False
            return True, False

        self._missing_count = 0
        self._candidate = candidate
        self._publish(candidate)

        if self.junction_y >= self.y2:
            # 红线越过 y2，触发转弯
            self.state = self.TURN_NOW
            if self.junction_y > self.y2:
                self._mark_passed()
            return True, True

        return True, False

    def _update_passing(self, binary_np, scan_result):
        if self.search_mode == self.SEARCH_NEXT:
            return self._search_next(binary_np, scan_result)

        candidate = self._find_gap(binary_np, scan_result)
        if candidate is not None and self._same_candidate(candidate, self._candidate):
            self._missing_count = 0
            self._candidate = candidate
            self._publish(candidate)
            if self.junction_y > self.y2:
                self._mark_passed()
        else:
            self._missing_count += 1
            if self._missing_count > self.hold_frames:
                self._reset_current()
                return False, False

        return True, False

    def _search_next(self, binary_np, scan_result):
        candidate = self._find_gap(binary_np, scan_result, max_line_y=self.y1 - 1)
        if candidate is None:
            self._new_candidate = None
            self._new_confirm_count = 0
            return True, False

        if self._same_candidate(candidate, self._new_candidate):
            self._new_confirm_count += 1
        else:
            self._new_confirm_count = 1
        self._new_candidate = candidate

        if self._new_confirm_count < self.new_confirm_frames:
            return True, False

        # 确认新路口，切换到 APPROACH
        self._candidate = candidate
        self._publish(candidate)
        self.state = self.APPROACH
        self.search_mode = self.TRACK_CURRENT
        self._confirm_count = self.confirm_frames
        self._missing_count = 0
        self._new_candidate = None
        self._new_confirm_count = 0
        return True, False

    # ====================  缺口检测  ====================

    def _find_gap(self, binary_np, scan_result, max_line_y=None):
        if scan_result is None or len(binary_np.shape) != 2:
            return None

        x_left, x_right, _, valid = scan_result
        H, W = binary_np.shape
        if len(valid) != H:
            return None

        anchor_y = self._find_anchor(valid, max_line_y)
        if anchor_y < 0:
            return None

        rows = binary_np.tobytes()
        predicted_left = x_left[anchor_y]
        predicted_right = x_right[anchor_y]
        last_normal = (anchor_y, predicted_left, predicted_right)
        gap_rows = 0

        for y in range(anchor_y - 1, -1, -1):
            row_start = y * W
            left = self._known_or_find_left(
                rows, row_start, W, predicted_left, x_left[y], valid[y]
            )
            right = self._known_or_find_right(
                rows, row_start, W, predicted_right, x_right[y], valid[y]
            )

            left_open = left < 0 and self._is_open(
                rows, row_start, W, predicted_left, True
            )
            right_open = right < 0 and self._is_open(
                rows, row_start, W, predicted_right, False
            )

            if left_open or right_open:
                gap_rows += 1
                if left >= 0:
                    predicted_left = left
                if right >= 0:
                    predicted_right = right
                if gap_rows >= self.min_gap_rows:
                    if max_line_y is None or last_normal[0] <= max_line_y:
                        return last_normal
                continue

            gap_rows = 0
            if left >= 0 and right >= 0 and right - left >= self.min_lane_width:
                predicted_left = left
                predicted_right = right
                last_normal = (y, left, right)

        return None

    # ====================  锚点与边缘查找  ====================

    @staticmethod
    def _find_anchor(valid, max_line_y):
        H = len(valid)
        if max_line_y is None:
            for y in range(H - 1, -1, -1):
                if valid[y]:
                    return y
            return -1

        # 从 y1 下方最近的有效行向上扫描
        start = max(0, max_line_y + 1)
        for y in range(start, H):
            if valid[y]:
                return y
        return -1

    def _known_or_find_left(self, data, row_start, width, predicted, known, is_valid):
        if is_valid and abs(known - predicted) <= self.probe_width:
            return known
        return self._find_left(data, row_start, width, predicted)

    def _known_or_find_right(self, data, row_start, width, predicted, known, is_valid):
        if is_valid and abs(known - predicted) <= self.probe_width:
            return known
        return self._find_right(data, row_start, width, predicted)

    def _find_left(self, data, row_start, width, predicted):
        for distance in range(self.probe_width + 1):
            x = predicted - distance
            if self._is_left_edge(data, row_start, width, x):
                return x
            if distance:
                x = predicted + distance
                if self._is_left_edge(data, row_start, width, x):
                    return x
        return -1

    def _find_right(self, data, row_start, width, predicted):
        for distance in range(self.probe_width + 1):
            x = predicted + distance
            if self._is_right_edge(data, row_start, width, x):
                return x
            if distance:
                x = predicted - distance
                if self._is_right_edge(data, row_start, width, x):
                    return x
        return -1

    def _is_left_edge(self, data, row_start, width, x):
        if x < self.black_run or x >= width:
            return False
        if data[row_start + x] < self.white_th:
            return False
        for offset in range(1, self.black_run + 1):
            if data[row_start + x - offset] > self.black_th:
                return False
        return True

    def _is_right_edge(self, data, row_start, width, x):
        if x < 0 or x + self.black_run >= width:
            return False
        if data[row_start + x] < self.white_th:
            return False
        for offset in range(1, self.black_run + 1):
            if data[row_start + x + offset] > self.black_th:
                return False
        return True

    def _is_open(self, data, row_start, width, predicted, is_left):
        # 从预测边界向外搜索，找到连续 white_run 个白像素即判定开放
        if is_left:
            x_range = range(predicted - 1, -1, -1)
        else:
            x_range = range(predicted + 1, width)

        run = 0
        for x in x_range:
            if data[row_start + x] >= self.white_th:
                run += 1
                if run >= self.white_run:
                    return True
            else:
                run = 0
        return False

    # ====================  辅助方法  ====================

    def _same_candidate(self, current, previous):
        return (
            previous is not None and abs(current[0] - previous[0]) <= self.track_max_dy
        )

    def _publish(self, candidate):
        self.junction_y = candidate[0]
        self.junction_left = candidate[1]
        self.junction_right = candidate[2]

    def _mark_passed(self):
        self.search_mode = self.SEARCH_NEXT
        self._new_candidate = None
        self._new_confirm_count = 0

    def _reset_current(self):
        self.state = self.NORMAL
        self.search_mode = self.TRACK_CURRENT
        self.junction_y = -1
        self.junction_left = -1
        self.junction_right = -1
        self._candidate = None
        self._confirm_count = 0
        self._missing_count = 0
        self._new_candidate = None
        self._new_confirm_count = 0


# ====================  绘图辅助  ====================


def draw_junction_overlay(overlay, detector, color=(255, 0, 0), thickness=1):
    """绘制当前跟踪的路口红线。"""
    if detector.junction_y < 0:
        return overlay
    import cv2

    cv2.line(
        overlay,
        (detector.junction_left, detector.junction_y),
        (detector.junction_right, detector.junction_y),
        color,
        thickness,
    )
    return overlay


def draw_junction_guides(overlay, y1, y2, color=(255, 255, 0), thickness=1):
    """绘制 y1、y2 两条水平参考线。"""
    import cv2

    height, width = overlay.shape[:2]
    for y in (y1, y2):
        if 0 <= y < height:
            cv2.line(overlay, (0, y), (width - 1, y), color, thickness)
    return overlay
