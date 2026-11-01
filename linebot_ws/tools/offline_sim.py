#!/usr/bin/env python3
"""Gazebo-free closed-loop check of the vision + PID stack.

Renders what the downward camera would see (same geometry as the URDF), runs the REAL
LineTracker + PID + control law at 30 Hz on a unicycle model with the same acceleration
limits as the Gazebo diff-drive plugin, and reports cross-track error.  Handy for first-pass
tuning before you open Gazebo.   Needs only numpy + opencv + pyyaml.

    python3 tools/offline_sim.py --track oval
    python3 tools/offline_sim.py --all
    python3 tools/offline_sim.py --track s_curve --kp 3 --kd 0.3 --max-speed 0.3 --save out.png
"""
import argparse
import importlib.util
import math
import os
import sys

import cv2
import numpy as np

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.join(ROOT, 'src', 'linebot_control'))
from linebot_control.control_law import command_from_error      # noqa: E402
from linebot_control.pid import PID                              # noqa: E402
from linebot_control.vision import LineTracker, TrackerConfig    # noqa: E402

spec = importlib.util.spec_from_file_location(
    'gen', os.path.join(ROOT, 'src', 'linebot_gazebo', 'scripts', 'generate_track.py'))
gen = importlib.util.module_from_spec(spec)
spec.loader.exec_module(gen)

# ---- camera geometry: keep in sync with linebot.urdf.xacro -------------
W, H = 640, 480
HFOV = 1.2
CAM_X, CAM_H, PITCH = 0.12, 0.12 + 0.035, 0.9     # m ahead of axle, m above ground, rad down
RES = 0.005                                         # m / px of the top-down floor raster
FPS, DT = 30, 1.0 / 30


def make_floor(pts, closed):
    pts = np.array(pts + ([pts[0]] if closed else []))
    pad = 3.0
    x0, y0 = pts[:, 0].min() - pad, pts[:, 1].min() - pad
    w = int((pts[:, 0].max() - x0 + pad) / RES)
    h = int((pts[:, 1].max() - y0 + pad) / RES)
    floor = np.full((h, w), 232, np.uint8)
    px = np.round((pts - [x0, y0]) / RES).astype(np.int32)
    cv2.polylines(floor, [px], False, 8, max(1, int(round(gen.LINE_WIDTH / RES))))
    return floor, x0, y0


def ray_table():
    fx = (W / 2) / math.tan(HFOV / 2)
    u, v = np.meshgrid(np.arange(W) + 0.5, np.arange(H) + 0.5)
    xo, yo = (u - W / 2) / fx, (v - H / 2) / fx
    xc, yc, zc = np.ones_like(xo), -xo, -yo                # optical -> camera_link
    c, s = math.cos(PITCH), math.sin(PITCH)
    xb, zb = c * xc + s * zc, -s * xc + c * zc             # pitch about +y (nose down)
    ok = zb < -1e-6
    t = np.where(ok, -CAM_H / np.where(ok, zb, -1), 0)
    return ok, CAM_X + t * xb, t * yc                      # ground x,y in base frame


RAYS = ray_table()


def render(floor, ox, oy, pose, rng):
    ok, gx, gy = RAYS
    x, y, yaw = pose
    wx = x + gx * math.cos(yaw) - gy * math.sin(yaw)
    wy = y + gx * math.sin(yaw) + gy * math.cos(yaw)
    ix = np.clip(((wx - ox) / RES).astype(int), 0, floor.shape[1] - 1)
    iy = np.clip(((wy - oy) / RES).astype(int), 0, floor.shape[0] - 1)
    img = np.where(ok, floor[iy, ix], 232).astype(np.int16)
    img += rng.normal(0, 3, img.shape).astype(np.int16)
    img = np.clip(img, 0, 255).astype(np.uint8)
    return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)


def run(track, args, save=None):
    pts, closed = gen.SHAPES[track]()
    pts = gen.resample(pts, 0.01, closed)
    floor, ox, oy = make_floor(pts, closed)
    path = np.array(pts)
    poses = {}
    import yaml
    with open(os.path.join(ROOT, 'src', 'linebot_gazebo', 'config', 'start_poses.yaml')) as f:
        poses = yaml.safe_load(f)
    p0 = poses[track]
    pose = [p0['x'], p0['y'], p0['yaw']]

    cfg = dict(max_speed=args.max_speed, min_speed=args.min_speed, slow_gain=args.slow_gain)
    pid = PID(args.kp, args.ki, args.kd, -args.max_angular, args.max_angular, 1.0, args.d_filter)
    tracker = LineTracker(TrackerConfig())
    rng = np.random.default_rng(0)

    v = w = 0.0
    xs, errs, lost_frames = [], [], 0
    length = float(np.sum(np.hypot(*np.diff(path, axis=0).T)))
    travelled, tracking, last_sign = 0.0, False, 1.0
    t, T = 0.0, args.seconds
    while t < T:
        img = render(floor, ox, oy, pose, rng)
        res = tracker.process(img)
        if res.detected:
            if not tracking:
                pid.reset(); tracking = True
            cv, cw = command_from_error(pid, res.error, res.far_error, cfg, DT)
            tv, tw = cv, cw
            if abs(res.error) > 0.02:
                last_sign = 1.0 if res.error > 0 else -1.0
        else:
            tracking = False; lost_frames += 1
            tv, tw = 0.0, -last_sign * 0.8                  # same as node's search behaviour
        # diff-drive plugin acceleration limits (2 m/s^2, 8 rad/s^2)
        v += max(-2 * DT, min(2 * DT, tv - v))
        w += max(-8 * DT, min(8 * DT, tw - w))
        pose[0] += v * math.cos(pose[2]) * DT
        pose[1] += v * math.sin(pose[2]) * DT
        pose[2] += w * DT
        travelled += v * DT
        # cross-track error of the axle to the path
        d = np.min(np.hypot(path[:, 0] - pose[0], path[:, 1] - pose[1]))
        xs.append(tuple(pose[:2])); errs.append(d)
        t += DT
        if lost_frames > 3 * FPS:
            break
        if closed and travelled > 1.05 * length:
            break
        if not closed and travelled > 0.97 * length:
            break

    errs = np.array(errs)
    done = travelled >= (0.97 if not closed else 1.0) * length
    print(f'{track:13s} {"OK  " if done and lost_frames <= 3*FPS else "FAIL"} '
          f'dist {travelled:5.1f}/{length:5.1f} m  t={t:5.1f}s  '
          f'avg v={travelled / max(t, 1e-6):.2f} m/s  '
          f'|xte| mean {errs.mean() * 100:4.1f} cm  max {errs.max() * 100:4.1f} cm  '
          f'lost-frames {lost_frames}')
    if save:
        vis = cv2.cvtColor(floor, cv2.COLOR_GRAY2BGR)
        for a in xs[::3]:
            cv2.circle(vis, (int((a[0] - ox) / RES), int((a[1] - oy) / RES)), 2, (0, 0, 255), -1)
        cv2.imwrite(save, cv2.flip(vis, 0))
    return done and lost_frames <= 3 * FPS


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--track', default='oval', choices=sorted(gen.SHAPES))
    ap.add_argument('--all', action='store_true')
    ap.add_argument('--kp', type=float, default=2.0)
    ap.add_argument('--ki', type=float, default=0.0)
    ap.add_argument('--kd', type=float, default=0.2)
    ap.add_argument('--d-filter', type=float, default=0.7)
    ap.add_argument('--max-angular', type=float, default=2.0)
    ap.add_argument('--max-speed', type=float, default=0.25)
    ap.add_argument('--min-speed', type=float, default=0.08)
    ap.add_argument('--slow-gain', type=float, default=0.6)
    ap.add_argument('--seconds', type=float, default=120.0)
    ap.add_argument('--save')
    a = ap.parse_args()
    tracks = sorted(gen.SHAPES) if a.all else [a.track]
    ok = all([run(t, a, a.save if not a.all else None) for t in tracks])
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
