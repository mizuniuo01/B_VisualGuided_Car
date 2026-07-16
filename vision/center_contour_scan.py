"""
center_contour_scan.py - 赛道轮廓扫线

输入约定：255 = 白色赛道，0 = 黑色背景
从图像底部向上逐行扫描，找出每行的左右边界和中心点
"""

import cv2

# ====================  扫线器  ====================


class CenterContourScanner:

    def __init__(
        self,
        row_step=4,
        black_run=3,
        white_th=200,
        black_th=50,
        min_width=20,
        center_search_radius=30,
        scan_offset=5,
    ):
        if black_run < 1:
            raise ValueError("black_run must be >= 1")
        self.row_step = row_step
        self.black_run = black_run
        self.white_th = white_th
        self.black_th = black_th
        self.min_width = min_width
        self.center_search_radius = center_search_radius
        self.scan_offset = scan_offset

    def __call__(self, binary_np):
        return self.apply(binary_np)

    # ----------  主入口  ----------

    def apply(self, binary_np):
        """返回 (x_left, x_right, x_center, valid)，长度均为 H。"""
        H, W = binary_np.shape
        x_left = [-1] * H
        x_right = [-1] * H
        x_center = [-1] * H
        valid = [False] * H

        binary_bytes = binary_np.tobytes()
        cx = W // 2
        prev_left = -1
        prev_right = -1

        for y in range(H - 1, -1, -self.row_step):
            row_start = y * W
            row = binary_bytes[row_start : row_start + W]

            left_start = self._left_start_x(row, cx, prev_left)
            right_start = self._right_start_x(row, cx, prev_right)

            left = self._scan_left(row, left_start) if left_start >= 0 else -1
            right = self._scan_right(row, right_start) if right_start >= 0 else -1

            prev_left = left if left >= 0 else -1
            prev_right = right if right >= 0 else -1

            if left < 0 or right < 0:
                continue

            width = right - left
            if width < self.min_width:
                # 宽度不足，重置上一帧引导
                prev_left = -1
                prev_right = -1
                continue

            x_left[y] = left
            x_right[y] = right
            x_center[y] = (left + right) // 2
            valid[y] = True

        return x_left, x_right, x_center, valid

    # ----------  起始点确定  ----------

    def _find_start_x(self, row, cx):
        """从图像中心向左右搜索白色起始点。"""
        if row[cx] >= self.white_th:
            return cx
        W = len(row)
        max_radius = min(self.center_search_radius, max(cx, W - 1 - cx))
        for offset in range(1, max_radius + 1):
            xl = cx - offset
            xr = cx + offset
            if xl >= 0 and row[xl] >= self.white_th:
                return xl
            if xr < W and row[xr] >= self.white_th:
                return xr
        return -1

    def _left_start_x(self, row, cx, prev_left):
        """左边界起始点：优先沿上一行边界偏移，否则从中心搜索。"""
        if prev_left >= 0:
            return min(prev_left + self.scan_offset, len(row) - 1)
        return self._find_start_x(row, cx)

    def _right_start_x(self, row, cx, prev_right):
        """右边界起始点：优先沿上一行边界偏移，否则从中心搜索。"""
        if prev_right >= 0:
            return max(prev_right - self.scan_offset, 0)
        return self._find_start_x(row, cx)

    # ----------  边界扫描  ----------

    def _scan_left(self, row, start_x):
        """从 start_x 向左扫，返回白→黑跳变处的最后一个白色像素坐标。"""
        white_th = self.white_th
        black_th = self.black_th
        black_run = self.black_run
        last_white = -1

        for x in range(start_x, -1, -1):
            if row[x] >= white_th:
                last_white = x
                continue
            if last_white < 0:
                continue
            # 确认连续 black_run 个黑像素
            ok = all((x - i) >= 0 and row[x - i] <= black_th for i in range(black_run))
            if ok:
                return last_white

        return -1

    def _scan_right(self, row, start_x):
        """从 start_x 向右扫，返回白→黑跳变处的最后一个白色像素坐标。"""
        white_th = self.white_th
        black_th = self.black_th
        black_run = self.black_run
        W = len(row)
        last_white = -1

        for x in range(start_x, W):
            if row[x] >= white_th:
                last_white = x
                continue
            if last_white < 0:
                continue
            # 确认连续 black_run 个黑像素
            ok = all((x + i) < W and row[x + i] <= black_th for i in range(black_run))
            if ok:
                return last_white

        return -1


# ====================  调试绘制  ====================


def draw_scan_overlay(
    overlay,
    x_left,
    x_right,
    x_center,
    valid,
    left_color=(0, 255, 0),
    right_color=(0, 0, 255),
    center_color=(255, 0, 0),
    point_radius=1,
):
    """在 RGB numpy 图像上绘制有效轮廓点。"""
    H = len(valid)
    for y in range(H):
        if not valid[y]:
            continue
        cv2.circle(overlay, (x_left[y], y), point_radius, left_color, -1)
        cv2.circle(overlay, (x_right[y], y), point_radius, right_color, -1)
        cv2.circle(overlay, (x_center[y], y), point_radius, center_color, -1)
    return overlay


# ====================  自测  ====================

if __name__ == "__main__":
    import numpy as np

    # 用例1：标准赛道，检查边界和中心坐标
    H, W = 80, 120
    binary = np.zeros((H, W), dtype=np.uint8)
    binary[:, 35:85] = 255
    scanner = CenterContourScanner(row_step=5, min_width=10)
    xl, xr, xc, v = scanner(binary)
    assert sum(v) == 16
    assert xl[H - 1] == 35
    assert xr[H - 1] == 84
    assert xc[H - 1] == 59

    # 用例2：边界噪点，检查 scan_offset 的引导效果
    H, W = 6, 80
    binary = np.zeros((H, W), dtype=np.uint8)
    binary[:, 12:60] = 255
    binary[5, 0:60] = 255
    binary[3, 11:60] = 255
    scanner = CenterContourScanner(row_step=1, min_width=10, scan_offset=5)
    xl, xr, xc, v = scanner(binary)
    assert not v[5]
    assert v[4] and xl[4] == 12
    assert v[3] and xl[3] == 11

    print("center_contour_scan self-test passed")
