"""Black-line tracking with OpenCV (no ROS dependency, so it is unit-testable).

The image is cropped to a region of interest (ROI) in the lower part of the frame,
thresholded, cleaned up with morphology, and split into horizontal slices.
Slice 0 is the NEAREST one (bottom of the image). In every slice the blob whose
centroid is closest to the previously tracked x position is taken as "the line"
(this keeps us locked on the right line at junctions / near side clutter).

Lateral error is normalised: 0 = centre, +1 = right edge, -1 = left edge.
"""
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

import cv2
import numpy as np


@dataclass
class TrackerConfig:
    roi_top: float = 0.35              # ROI start, fraction of image height
    roi_bottom: float = 1.0            # ROI end, fraction of image height
    slice_weights: Sequence[float] = (0.5, 0.3, 0.2)   # nearest slice first
    threshold_mode: str = 'fixed'      # 'fixed' | 'otsu' | 'adaptive'
    threshold: int = 80                # used by 'fixed' (pixels darker than this = line)
    blur_kernel: int = 5
    morph_kernel: int = 5
    min_area: float = 120.0            # px^2, blobs smaller than this are ignored
    max_slice_jump: float = 0.30       # max x-jump between slices, fraction of image width


@dataclass
class SliceResult:
    index: int          # 0 = nearest
    cx: float           # centroid, full-frame pixel coordinates
    cy: float
    error: float        # normalised lateral error


@dataclass
class TrackResult:
    detected: bool = False
    error: float = 0.0
    near_error: float = 0.0
    far_error: float = 0.0
    slices: List[SliceResult] = field(default_factory=list)
    mask: Optional[np.ndarray] = None     # binary mask, ROI-sized
    roi: Tuple[int, int] = (0, 0)         # (y0, y1) in full-frame rows
    n_slices: int = 0


def _odd(k: int) -> int:
    k = int(k)
    return k if k % 2 == 1 else k + 1 if k > 0 else 0


class LineTracker:
    def __init__(self, cfg: Optional[TrackerConfig] = None):
        self.cfg = cfg or TrackerConfig()
        self._last_cx: Optional[float] = None

    # ------------------------------------------------------------------
    def _threshold(self, roi_gray: np.ndarray) -> np.ndarray:
        mode = self.cfg.threshold_mode
        if mode == 'otsu':
            # Otsu on a featureless ROI splits noise -> only trust it with real contrast.
            if int(roi_gray.max()) - int(roi_gray.min()) < 40:
                return np.zeros_like(roi_gray)
            _, m = cv2.threshold(roi_gray, 0, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)
        elif mode == 'adaptive':
            m = cv2.adaptiveThreshold(roi_gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                      cv2.THRESH_BINARY_INV, 51, 20)
        else:
            _, m = cv2.threshold(roi_gray, int(self.cfg.threshold), 255, cv2.THRESH_BINARY_INV)
        return m

    # ------------------------------------------------------------------
    def process(self, frame: np.ndarray) -> TrackResult:
        cfg = self.cfg
        h, w = frame.shape[:2]

        y0 = int(np.clip(cfg.roi_top, 0.0, 0.95) * h)
        y1 = int(np.clip(cfg.roi_bottom, 0.05, 1.0) * h)
        if y1 - y0 < 8:
            y0, y1 = 0, h

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
        roi = gray[y0:y1]

        k = _odd(cfg.blur_kernel)
        if k > 1:
            roi = cv2.GaussianBlur(roi, (k, k), 0)

        mask = self._threshold(roi)

        mk = _odd(cfg.morph_kernel)
        if mk > 1:
            ker = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (mk, mk))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, ker)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, ker)

        weights = [max(0.0, float(x)) for x in cfg.slice_weights] or [1.0]
        n = len(weights)
        roi_h = y1 - y0
        sh = max(1, roi_h // n)
        half_w = w / 2.0

        slices: List[SliceResult] = []
        ref = self._last_cx if self._last_cx is not None else half_w

        for i in range(n):                       # i = 0 -> bottom (nearest) slice
            ye = roi_h - i * sh
            ys = 0 if i == n - 1 else max(0, ye - sh)
            if ye <= ys:
                continue
            strip = mask[ys:ye]
            contours, _ = cv2.findContours(strip, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            best, best_d = None, float('inf')
            for c in contours:
                m = cv2.moments(c)
                if m['m00'] < cfg.min_area:
                    continue
                cx = m['m10'] / m['m00']
                cy = m['m01'] / m['m00'] + ys
                d = abs(cx - ref)
                if slices and d > cfg.max_slice_jump * w:   # ignore far-off branches
                    continue
                if d < best_d:
                    best_d, best = d, (cx, cy)

            if best is not None:
                cx, cy = best
                slices.append(SliceResult(i, cx, cy + y0, (cx - half_w) / half_w))
                ref = cx

        res = TrackResult(mask=mask, roi=(y0, y1), n_slices=n)
        if slices:
            wts = [weights[s.index] for s in slices]
            wsum = sum(wts)
            if wsum > 0:
                res.error = sum(wt * s.error for wt, s in zip(wts, slices)) / wsum
            else:
                res.error = sum(s.error for s in slices) / len(slices)
            res.near_error = slices[0].error
            res.far_error = slices[-1].error
            res.detected = True
            res.slices = slices
            self._last_cx = slices[0].cx
        return res

    # ------------------------------------------------------------------
    @staticmethod
    def draw_debug(frame: np.ndarray, res: TrackResult) -> np.ndarray:
        dbg = frame.copy()
        h, w = dbg.shape[:2]
        y0, y1 = res.roi
        if res.mask is not None and y1 > y0:
            dbg[y0:y1][res.mask > 0] = (0, 0, 255)           # line pixels in red
        cv2.rectangle(dbg, (0, y0), (w - 1, y1 - 1), (0, 255, 255), 1)
        cv2.line(dbg, (w // 2, y0), (w // 2, y1), (255, 255, 0), 1)   # image centre
        pts = [(int(s.cx), int(s.cy)) for s in res.slices]
        for a, b in zip(pts, pts[1:]):
            cv2.line(dbg, a, b, (0, 255, 0), 2)
        for p in pts:
            cv2.circle(dbg, p, 6, (0, 255, 0), -1)
        label = (f'err={res.error:+.2f} near={res.near_error:+.2f} far={res.far_error:+.2f}'
                 if res.detected else 'LINE LOST')
        cv2.putText(dbg, label, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (0, 255, 0) if res.detected else (0, 0, 255), 2)
        return dbg
