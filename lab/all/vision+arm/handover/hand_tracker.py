#!/usr/bin/env python3
"""
hand_tracker.py — 手指骨骼版 (v5)

演算法:
  1. rs.align 硬體對齊 depth→color
  2. HSV 膚色遮罩 (排除非皮膚)
  3. 在膚色區域內找最近深度點 → 種子 (手最突出)
  4. 種子生長 5cm → 手部區域 (砍掉臉/身體)
  5. 輪廓 → 凸包 → 凸性缺陷 = 手指指尖
  6. 指尖 = 骨骼端點 → 手掌中心 = 算術中點

CAM1 (D435I S/N 244222075086):
  - color: 848x480, depth: 424x240, aligned depth: 848x480
"""

import cv2
import numpy as np
import pyrealsense2 as rs

CAM_SN = "244222075086"


class HandTracker:
    def __init__(self, sn=CAM_SN, width=848, height=480, fps=15):
        self.width = width
        self.height = height
        self.sn = sn
        self.depth_w = 424
        self.depth_h = 240

        self.pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_device(sn)
        cfg.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        cfg.enable_stream(rs.stream.depth, self.depth_w, self.depth_h, rs.format.z16, fps)

        try:
            profile = self.pipe.start(cfg)
            print(f"[HandTracker] ✅ CAM1 ({sn}) @ color={width}x{height} depth={self.depth_w}x{self.depth_h} {fps}fps")

            depth_sensor = profile.get_device().first_depth_sensor()
            self.depth_scale = depth_sensor.get_depth_scale()

            color_stream = profile.get_stream(rs.stream.color)
            self.color_intr = color_stream.as_video_stream_profile().get_intrinsics()
            depth_stream = profile.get_stream(rs.stream.depth)
            self.depth_intr = depth_stream.as_video_stream_profile().get_intrinsics()

            # rs.align 硬體對齊 depth → color
            self.align = rs.align(rs.stream.color)

            hfov = 2 * (width / 2 / self.color_intr.fx) * 180 / np.pi
            print(f"[HandTracker]    col: {width}x{height} fx={self.color_intr.fx:.1f} HFOV={hfov:.0f}°")
            print(f"[HandTracker]    dep: {self.depth_w}x{self.depth_h} fx={self.depth_intr.fx:.1f}")
            print(f"[HandTracker]    align: depth→color (rs.align)")

        except Exception as e:
            print(f"[HandTracker] ❌ 启动失败: {e}")
            raise

        for _ in range(8):
            try:
                self.pipe.wait_for_frames(timeout_ms=3000)
            except RuntimeError:
                pass

        self._sx = self._sy = None
        self._lost = 0

    def get_frames(self, retries=3):
        for attempt in range(retries):
            try:
                frames = self.pipe.wait_for_frames(timeout_ms=5000)
                # 硬體對齊
                aligned = self.align.process(frames)
                color = aligned.get_color_frame()
                depth = aligned.get_depth_frame()
                if not color or not depth:
                    return None, None, None
                color_img = np.asanyarray(color.get_data())
                depth_img = np.asanyarray(depth.get_data())
                raw_frames = frames
                return color_img, depth_img, raw_frames
            except RuntimeError:
                if attempt < retries - 1:
                    pass
                else:
                    return None, None, None

    @staticmethod
    def _is_skin_color_hsv(bgr):
        hsv = cv2.cvtColor(np.uint8([[bgr]]), cv2.COLOR_BGR2HSV)[0, 0]
        h, s, v = int(hsv[0]), int(hsv[1]), int(hsv[2])
        return (30 <= s <= 170) and (60 <= v <= 250) and (h <= 30 or h >= 150)

    def _get_skin_mask(self, color_img):
        """HSV 膚色遮罩, 回傳 uint8 mask (848x480)"""
        hsv = cv2.cvtColor(color_img, cv2.COLOR_BGR2HSV)
        lower1 = np.array([0, 30, 60])
        upper1 = np.array([30, 170, 250])
        lower2 = np.array([150, 30, 60])
        upper2 = np.array([180, 170, 250])
        skin1 = cv2.inRange(hsv, lower1, upper1)
        skin2 = cv2.inRange(hsv, lower2, upper2)
        skin = cv2.bitwise_or(skin1, skin2)
        # 形態學: 關閉小孔 + 平滑
        skin = cv2.morphologyEx(skin, cv2.MORPH_CLOSE, None, iterations=2)
        skin = cv2.erode(skin, None, iterations=1)
        skin = cv2.dilate(skin, None, iterations=2)
        return skin

    def _analyze_fingers(self, contour):
        """找指尖和手掌中心

        回傳: (指尖列表, 手掌中心, 凸性缺陷數)
          - 指尖列表: [(x,y), ...] (從指尖到虎口/小指)
          - 手掌中心: (x,y) 凸包質心
          - 缺陷數: 手指谷底數
        """
        if contour is None or len(contour) < 10:
            return [], None, 0

        hull = cv2.convexHull(contour, returnPoints=False)
        if hull is None or len(hull) < 5:
            return [], None, 0

        # 凸性缺陷
        defects = cv2.convexityDefects(contour, hull)
        if defects is None:
            return [], None, 0

        # 分析缺陷 → 找指尖
        # 缺陷是輪廓上偏離凸包的點(谷底)，兩個缺陷之間的凸包頂點是指尖
        n_defects = len(defects)

        # 提取所有凸包頂點 (可能的指尖)
        hull_pts = cv2.convexHull(contour, returnPoints=True)
        hull_idx = hull.flatten()

        # 對每個凸包頂點, 如果兩側都有較深的缺陷 → 是指尖
        # 簡單法: 取所有凸包頂點, 過濾邊緣頂點
        fingers = []
        for i in range(len(hull_pts)):
            # 取遠離輪廓質心的頂點 = 指尖
            pt = hull_pts[i][0]
            dist = cv2.pointPolygonTest(contour, (float(pt[0]), float(pt[1])), True)
            # 正距離 = 多邊形外 = 凸包頂點突出 = 指尖
            if dist < 1:  # 幾乎在輪廓上, 不是突出點
                continue
            fingers.append(tuple(pt))

        # 手掌中心: 最大的內接圓中心 (距離變換)
        # 用整個手部輪廓計算距離變換
        h, w = 480, 848
        mask = np.zeros((h, w), dtype=np.uint8)
        cv2.drawContours(mask, [contour], -1, 255, -1)

        # 僅 on mask 的距離變換
        dt = cv2.distanceTransform(mask, cv2.DIST_L2, 3)
        _, _, _, max_loc = cv2.minMaxLoc(dt)
        palm_center = max_loc  # (x, y)

        # 指尖列表: 按到手掌中心的距離排序 (最遠 = 中指)
        if len(fingers) >= 2:
            fingers.sort(key=lambda p: np.hypot(p[0]-palm_center[0], p[1]-palm_center[1]), reverse=True)

        return fingers, palm_center, n_defects

    def _draw_skeleton(self, img, fingers, palm_center):
        """在圖上繪製骨骼和關節"""
        if palm_center is None:
            return img
        # 手掌中心 (紅色大圓)
        cv2.circle(img, palm_center, 10, (0, 0, 255), -1)
        cv2.circle(img, palm_center, 12, (255, 255, 255), 2)

        if not fingers:
            return img

        for i, f in enumerate(fingers):
            # 指尖 (綠色圓)
            cv2.circle(img, f, 6, (0, 255, 0), -1)
            # 骨骼線 (從指尖連到手掌中心)
            cv2.line(img, f, palm_center, (255, 0, 255), 2)
            # 指尖編號
            cv2.putText(img, str(i), (f[0]+8, f[1]-4),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        return img

    def detect_hand(self, color_img, aligned_depth=None):
        """
        手指骨骼版檢測

        步驟:
          1. HSV 膚色遮罩
          2. 在膚色區域找最近深度點 → 種子
          3. 種子生長 5cm → 手部區域 (排除身體/臉)
          4. 輪廓 → 凸包 → 凸性缺陷 = 手指分析
          5. 回傳 (指尖, 手掌中心, 缺陷數)
        """
        if color_img is None:
            return [], None, 0

        h, w = color_img.shape[:2]

        # === Step 1-2: 膚色遮罩 + 深度種子 ===
        skin = self._get_skin_mask(color_img)

        if np.count_nonzero(skin) < 100:
            return [], None, 0

        # === Step 3: 在膚色區域內找最近深度 ===
        if aligned_depth is not None:
            depth_m = aligned_depth.astype(np.float32) * self.depth_scale

            # 只取膚色像素的深度
            skin_depths = depth_m.copy()
            skin_depths[skin == 0] = 0

            # 找到最近的有效膚色深度
            valid_skin = skin_depths[(skin_depths >= 0.15) & (skin_depths <= 1.5)]
            if valid_skin.size < 20:
                return [], None, 0

            z_min = float(np.min(valid_skin))
            z_max = z_min + 0.05  # 最近 5cm

            # 最前層遮罩: 膚色中最近 5cm
            fore_mask = (skin > 0) & (depth_m >= z_min) & (depth_m <= z_max)
            fore_mask = fore_mask.astype(np.uint8) * 255

            # 形態學清理
            fore_mask = cv2.erode(fore_mask, None, iterations=1)
            fore_mask = cv2.dilate(fore_mask, None, iterations=1)

            if np.count_nonzero(fore_mask) < 50:
                # 回退: 用整個皮膚遮罩
                fore_mask = skin
        else:
            fore_mask = skin

        # === Step 4: 輪廓分析 ===
        ctrs, _ = cv2.findContours(fore_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not ctrs:
            return [], None, 0

        # 取最大的輪廓 (手部區域)
        largest = max(ctrs, key=cv2.contourArea)
        area = cv2.contourArea(largest)
        if area < 300:
            return [], None, 0

        # === Step 5: 手指分析 ===
        fingers, palm_center, n_defects = self._analyze_fingers(largest)

        if palm_center is None:
            self._lost += 1
            if self._lost > 5:
                self._sx = self._sy = None
            return [], None, 0

        # 時序濾波
        cx, cy = palm_center
        if self._sx is None:
            self._sx, self._sy = float(cx), float(cy)
        else:
            jump = np.hypot(cx - self._sx, cy - self._sy)
            if jump < 200:
                alpha = 0.3
                self._sx = self._sx * (1 - alpha) + cx * alpha
                self._sy = self._sy * (1 - alpha) + cy * alpha
            else:
                self._sx, self._sy = float(cx), float(cy)

        self._lost = 0
        return fingers, (int(round(self._sx)), int(round(self._sy))), n_defects

    def get_3d_position(self, cx, cy, aligned_depth):
        """取手掌中心的 3D 位置"""
        if aligned_depth is None:
            return None
        cx = max(0, min(self.width - 1, cx))
        cy = max(0, min(self.height - 1, cy))
        half = 15
        y1 = max(0, cy - half)
        y2 = min(self.height, cy + half + 1)
        x1 = max(0, cx - half)
        x2 = min(self.width, cx + half + 1)
        patch = aligned_depth[y1:y2, x1:x2]
        if patch.size == 0:
            return None
        valid = patch[patch > 0]
        if valid.size < 30:
            return None
        depth_val = np.median(valid) * self.depth_scale
        if depth_val > 3.0 or depth_val < 0.1:
            return None
        x3d = (cx - self.color_intr.ppx) * depth_val / self.color_intr.fx
        y3d = (cy - self.color_intr.ppy) * depth_val / self.color_intr.fy
        return x3d, y3d, depth_val

    def draw_hand(self, color_img, fingers, palm_center, contour=None):
        img = color_img.copy()
        if palm_center:
            img = self._draw_skeleton(img, fingers, palm_center)
        if contour is not None:
            cv2.drawContours(img, [contour], -1, (0, 255, 0), 2)
        if palm_center:
            cv2.putText(img, f"PALM ({palm_center[0]},{palm_center[1]})",
                       (palm_center[0]+14, palm_center[1]-8),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            cv2.putText(img, f"Fingers: {len(fingers)}",
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)
        return img

    def stop(self):
        self.pipe.stop()


def test():
    t = HandTracker()
    try:
        while True:
            c, d, raw = t.get_frames()
            if c is None: continue
            fingers, palm, nd = t.detect_hand(c, d)

            display = c.copy()
            txt = "No hand"
            if palm:
                display = t.draw_hand(c, fingers, palm)
                pos = t.get_3d_position(palm[0], palm[1], d)
                if pos:
                    txt = f"Palm 3D: ({pos[0]:.2f},{pos[1]:.2f},{pos[2]:.2f})m Sk: {len(fingers)} def: {nd}"
                else:
                    txt = f"Palm ({palm[0]},{palm[1]}) Sk: {len(fingers)}"
            else:
                cv2.putText(display, "No hand", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

            cv2.imshow("Finger Bone Tracking", display)
            if cv2.waitKey(1) & 0xFF == 27:
                break
    finally:
        t.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    test()
