"""
Intelligence Layer #1 — Perception
입력: RGB(or BGR) 프레임 (numpy HxWx3)
출력: detection dict  {present, bbox, vis, landmarks, signals}

- Isaac Sim 불필요. 저장된 프레임(demo_frames/, scan/)으로 단독 테스트 가능.
- 백엔드 교체 가능: 현재 mediapipe pose. 확장 시 VLM/YOLO 등으로 detect() 시그니처 유지.

CODEX 확장 포인트:
- 다중 인물 탐지(현재 mediapipe pose는 1인) -> detect_multi()
- 얼굴 가림/소지품 등 추가 신호 추출
"""
from __future__ import annotations
import math
import numpy as np
import cv2
import mediapipe as mp

_mp_pose = mp.solutions.pose
_mp_draw = mp.solutions.drawing_utils


class PersonPerception:
    def __init__(self, min_det=0.4, min_track=0.4, complexity=1, det_threshold=0.3):
        self.pose = _mp_pose.Pose(static_image_mode=False, model_complexity=complexity,
                                  min_detection_confidence=min_det,
                                  min_tracking_confidence=min_track)
        self.det_threshold = det_threshold
        self._last_center_px = None

    def detect(self, frame_bgr) -> dict:
        """frame_bgr: HxWx3 BGR uint8. returns detection dict."""
        h, w = frame_bgr.shape[:2]
        res = self.pose.process(cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB))
        out = {"present": False, "bbox": None, "vis": 0.0,
               "landmarks": None, "signals": {}, "_raw": None}
        if not res.pose_landmarks:
            self._last_center_px = None
            return out
        lms = res.pose_landmarks.landmark
        xs = [l.x for l in lms]; ys = [l.y for l in lms]; vs = [l.visibility for l in lms]
        bbox = self._clamp_bbox((int(min(xs)*w), int(min(ys)*h), int(max(xs)*w), int(max(ys)*h)), w, h)
        vis = float(np.mean(vs))
        geometry = self._bbox_geometry(bbox, w, h)
        signals = self._pose_signals(lms)
        signals.update(geometry)
        signals.update(self._motion_signals(geometry["bbox_center_px"]))
        quality = self._observation_quality(vis, signals)
        present = quality >= self.det_threshold
        out.update(present=present, bbox=bbox, vis=vis,
                   landmarks=[(l.x, l.y, l.visibility) for l in lms],
                   signals=signals,
                   bbox_center=geometry["bbox_center_px"],
                   bbox_area=geometry["bbox_area_px"],
                   bbox_norm_area=geometry["bbox_norm_area"],
                   pose_quality=quality,
                   _raw=res.pose_landmarks)
        return out

    def detect_multi(self, frame_bgr) -> list[dict]:
        """Return a list of person detections.

        MediaPipe Pose tracks one body in this demo build, so this preserves a
        future-ready multi-person interface without changing detect().
        """
        det = self.detect(frame_bgr)
        if not det.get("present"):
            return []
        one = {k: v for k, v in det.items() if k != "_raw"}
        one["id"] = "person_0"
        return [one]

    @staticmethod
    def _clamp_bbox(bbox, w, h):
        x1, y1, x2, y2 = bbox
        x1 = max(0, min(w - 1, x1)); x2 = max(0, min(w - 1, x2))
        y1 = max(0, min(h - 1, y1)); y2 = max(0, min(h - 1, y2))
        return (min(x1, x2), min(y1, y2), max(x1, x2), max(y1, y2))

    @staticmethod
    def _bbox_geometry(bbox, w, h) -> dict:
        x1, y1, x2, y2 = bbox
        bw = max(0, x2 - x1)
        bh = max(0, y2 - y1)
        cx = x1 + bw * 0.5
        cy = y1 + bh * 0.5
        area = bw * bh
        norm_area = float(area / max(1, w * h))
        center_norm = (float(cx / max(1, w)), float(cy / max(1, h)))
        center_offset = math.hypot(center_norm[0] - 0.5, center_norm[1] - 0.5) / math.hypot(0.5, 0.5)
        edge_margin = min(cx, cy, w - cx, h - cy) / max(1.0, min(w, h) * 0.5)
        return {
            "bbox_center_px": (float(cx), float(cy)),
            "bbox_center_norm": center_norm,
            "bbox_area_px": int(area),
            "bbox_norm_area": norm_area,
            "bbox_aspect": float(bw / max(1, bh)),
            "center_offset": float(max(0.0, min(1.0, center_offset))),
            "edge_margin": float(max(0.0, min(1.0, edge_margin))),
            "near_frame_edge": edge_margin < 0.15,
        }

    @staticmethod
    def _pose_signals(lms) -> dict:
        # Face visibility is a weak proxy only; final demo judgement remains time/context-based.
        face_ids = [0, 1, 2, 3, 4, 5, 6, 7, 8]
        torso_ids = [11, 12, 23, 24]
        upper_body_ids = [11, 12, 13, 14, 15, 16]
        lower_body_ids = [23, 24, 25, 26, 27, 28]
        face_vis = float(np.mean([lms[i].visibility for i in face_ids]))
        torso_vis = float(np.mean([lms[i].visibility for i in torso_ids]))
        upper_vis = float(np.mean([lms[i].visibility for i in upper_body_ids]))
        lower_vis = float(np.mean([lms[i].visibility for i in lower_body_ids]))
        return {
            "face_visibility": face_vis,
            "face_occluded": face_vis < 0.25,
            "torso_visibility": torso_vis,
            "upper_body_visibility": upper_vis,
            "lower_body_visibility": lower_vis,
            "body_visibility": float(np.mean([torso_vis, upper_vis, lower_vis])),
            "partial_body": lower_vis < 0.25 or torso_vis < 0.35,
        }

    def _motion_signals(self, center_px) -> dict:
        if self._last_center_px is None:
            self._last_center_px = center_px
            return {"motion_px": 0.0, "stationary_frame": False}
        dx = center_px[0] - self._last_center_px[0]
        dy = center_px[1] - self._last_center_px[1]
        motion = float(math.hypot(dx, dy))
        self._last_center_px = center_px
        return {"motion_px": motion, "stationary_frame": motion < 3.0}

    @staticmethod
    def _observation_quality(vis, signals) -> float:
        size_score = min(1.0, signals["bbox_norm_area"] / 0.06)
        center_score = 1.0 - signals["center_offset"] * 0.35
        body_score = max(signals["torso_visibility"], signals["body_visibility"] * 0.75)
        edge_penalty = 0.20 if signals["near_frame_edge"] else 0.0
        quality = 0.45 * vis + 0.30 * body_score + 0.15 * size_score + 0.10 * center_score - edge_penalty
        return float(max(0.0, min(1.0, quality)))

    def draw_skeleton(self, frame_bgr, det):
        """탐지 결과의 스켈레톤을 프레임에 직접 그림(선택)."""
        if det.get("_raw") is not None:
            _mp_draw.draw_landmarks(frame_bgr, det["_raw"], _mp_pose.POSE_CONNECTIONS)
        return frame_bgr


# ---- 단독 테스트: 저장 프레임에 돌려보기 ----
if __name__ == "__main__":
    import sys, glob, os
    _here = os.path.dirname(os.path.abspath(__file__))
    paths = sys.argv[1:] or sorted(glob.glob(os.path.join(_here, "scan", "d600_f25.png")))
    pp = PersonPerception()
    for p in paths:
        img = cv2.imread(p)
        if img is None:
            print("skip", p); continue
        d = pp.detect(img)
        print(os.path.basename(p), "present=%s vis=%.2f quality=%.2f bbox=%s area=%.3f" %
              (d["present"], d["vis"], d.get("pose_quality", 0.0), d["bbox"], d.get("bbox_norm_area", 0.0)))
        print("  multi=%d signals=%s" % (len(pp.detect_multi(img)), d.get("signals", {})))
