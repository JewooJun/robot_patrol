"""
Intelligence Layer #2 — Suspicion (판단)
순수 로직. Isaac/카메라 불필요. 단위 테스트 가능.

신호 (확정 설계):
- 체류시간(핵심): 사람이 카메라에 연속 포착되며 seen_time 누적
- 공간/시간 맥락: 제한구역, 민감 perimeter, 업무시간 외, 핵심 구역 근접
- 행동 패턴: 배회/왕복/반복 방문 등 public space 안의 접근 의도 신호
- 얼굴 가림/비정상 위치: 선택 신호로 가중합 반영
- VLM 장면해석: assess_with_vlm() 훅만 제공, 데모 기본값은 규칙기반

레벨: normal -> watch -> suspicious
suspicious 도달 시 로봇이 접근/확인하도록 트리거.

CODEX 확장 포인트:
- weight 기반 다중 신호 합산
- VLM 호출 훅 (assess_with_vlm) — 현재는 규칙기반 fallback
"""
from __future__ import annotations


class SuspicionEngine:
    def __init__(self, suspect_secs=2.5, watch_secs=1.0, decay=0.7,
                 weights=None, use_vlm=False):
        self.suspect_secs = suspect_secs
        self.watch_secs = watch_secs
        self.decay = decay
        self.weights = weights or {
            "dwell": 0.52,
            "zone": 0.22,
            "behavior": 0.16,
            "time": 0.08,
            "face": 0.06,
            "vlm": 0.0,
        }
        self.use_vlm = use_vlm
        self.seen_time = 0.0
        self.level = "normal"
        self.triggered = False   # suspicious 트리거 1회성 플래그

    def update(self, detected: bool, dt: float) -> dict:
        signals = self._normalize_input(detected)
        present = bool(signals["detected"])

        if present:
            self.seen_time = min(self.suspect_secs + 1.0, self.seen_time + dt)
        else:
            self.seen_time = max(0.0, self.seen_time - dt * self.decay)

        dwell_score = self._clamp01(self.seen_time / max(self.suspect_secs, 0.001))
        face_score = 1.0 if signals["face_occluded"] else 0.0
        zone_score = self._zone_score(signals)
        behavior_score = self._behavior_score(signals)
        time_score = 1.0 if signals["after_hours"] else 0.0
        vlm_score = self.assess_with_vlm(signals) if self.use_vlm else 0.0
        score = min(1.0,
                    self.weights.get("dwell", 0.0) * dwell_score +
                    self.weights.get("zone", 0.0) * zone_score +
                    self.weights.get("behavior", 0.0) * behavior_score +
                    self.weights.get("time", 0.0) * time_score +
                    self.weights.get("face", 0.0) * face_score +
                    self.weights.get("vlm", 0.0) * vlm_score)

        reasons = self._reasons(signals, dwell_score, zone_score, behavior_score, time_score, face_score, vlm_score)

        # 체류시간은 데모의 설명 가능한 핵심 트리거로 유지한다.
        hard_context = (
            present and signals["restricted_location"]
            or present and signals["near_restricted_area"] and self.seen_time >= signals["loitering_threshold_sec"]
        )
        if self.seen_time >= self.suspect_secs or score >= 0.78 or hard_context:
            self.level = "suspicious"
        elif self.seen_time >= self.watch_secs or score >= 0.35:
            self.level = "watch"
        else:
            self.level = "normal"

        # 트리거는 normal/watch -> suspicious 전이에서 1회
        fire = False
        if self.level == "suspicious" and not self.triggered:
            self.triggered = True
            fire = True
        if self.level == "normal":
            self.triggered = False

        return {"seen_time": self.seen_time, "level": self.level,
                "score": score, "fire": fire,
                "reasons": reasons,
                "signals": {"dwell": dwell_score, "face": face_score,
                            "zone": zone_score, "behavior": behavior_score,
                            "time": time_score, "vlm": vlm_score,
                            "zone_id": signals["zone_id"],
                            "movement_pattern": signals["movement_pattern"]}}

    def reset_trigger(self):
        self.triggered = False

    @staticmethod
    def _normalize_input(detected) -> dict:
        if isinstance(detected, dict):
            signals = detected.get("signals", {}) or {}
            zone = detected.get("zone", signals.get("zone", {})) or {}
            return {
                "detected": detected.get("present", detected.get("detected", False)),
                "face_occluded": bool(detected.get("face_occluded", signals.get("face_occluded", False))),
                "restricted_location": bool(detected.get("restricted_location", signals.get("restricted_location", False))),
                "near_restricted_area": bool(detected.get("near_restricted_area", signals.get("near_restricted_area", False))),
                "after_hours": bool(detected.get("after_hours", signals.get("after_hours", False))),
                "zone_id": detected.get("zone_id", zone.get("id", signals.get("zone_id", ""))),
                "zone_type": detected.get("zone_type", zone.get("type", signals.get("zone_type", ""))),
                "access_level": detected.get("access_level", zone.get("access_level", signals.get("access_level", ""))),
                "loitering_allowed": bool(detected.get("loitering_allowed", zone.get("loitering_allowed", signals.get("loitering_allowed", True)))),
                "loitering_threshold_sec": float(detected.get("loitering_threshold_sec", zone.get("loitering_threshold_sec", signals.get("loitering_threshold_sec", 4.0)))),
                "zone_risk": float(detected.get("zone_risk", zone.get("risk_weight", signals.get("zone_risk", 0.0)))),
                "distance_to_restricted_cm": detected.get("distance_to_restricted_cm", signals.get("distance_to_restricted_cm", None)),
                "movement_pattern": str(detected.get("movement_pattern", detected.get("behavior", signals.get("movement_pattern", signals.get("behavior", ""))))),
                "repeat_visits": int(detected.get("repeat_visits", signals.get("repeat_visits", 0))),
                "pacing": bool(detected.get("pacing", signals.get("pacing", False))),
                "vlm_score": float(detected.get("vlm_score", signals.get("vlm_score", 0.0))),
                "raw": detected,
            }
        return {"detected": bool(detected), "face_occluded": False,
                "restricted_location": False, "near_restricted_area": False,
                "after_hours": False, "zone_id": "", "zone_type": "",
                "access_level": "", "loitering_allowed": True,
                "loitering_threshold_sec": 4.0, "zone_risk": 0.0,
                "distance_to_restricted_cm": None, "movement_pattern": "",
                "repeat_visits": 0, "pacing": False, "vlm_score": 0.0,
                "raw": detected}

    def _zone_score(self, signals) -> float:
        score = self._clamp01(signals["zone_risk"])
        if signals["restricted_location"] or signals["access_level"] == "restricted":
            score = max(score, 1.0)
        elif signals["zone_type"] == "sensitive_perimeter" or signals["near_restricted_area"]:
            score = max(score, 0.65)
        if not signals["loitering_allowed"]:
            score = max(score, 0.45)
        dist = signals.get("distance_to_restricted_cm")
        if isinstance(dist, (int, float)):
            if dist <= 200:
                score = max(score, 0.85)
            elif dist <= 500:
                score = max(score, 0.65)
        return self._clamp01(score)

    def _behavior_score(self, signals) -> float:
        pat = signals["movement_pattern"].lower()
        score = 0.0
        if pat in ("idle", "standing", "loiter", "loitering"):
            score = 0.45
        elif pat in ("pace", "pacing"):
            score = 0.75
        elif pat in ("wander", "wandering"):
            score = 0.65
        elif pat in ("loop", "repeated_loop"):
            score = 0.40
        elif pat in ("run", "running", "flee"):
            score = 0.85
        if signals["pacing"]:
            score = max(score, 0.75)
        if signals["repeat_visits"] >= 3:
            score = max(score, 0.70)
        return self._clamp01(score)

    def _reasons(self, signals, dwell, zone, behavior, time_, face, vlm):
        reasons = []
        if dwell >= 1.0:
            reasons.append("loitering_threshold_exceeded")
        elif dwell >= 0.4:
            reasons.append("continued_presence")
        if signals["restricted_location"]:
            reasons.append("inside_restricted_area")
        elif signals["zone_type"] == "sensitive_perimeter" or signals["near_restricted_area"]:
            reasons.append("near_sensitive_perimeter")
        if not signals["loitering_allowed"]:
            reasons.append("loitering_not_allowed_here")
        if time_ > 0:
            reasons.append("after_hours")
        if behavior >= 0.7:
            reasons.append("suspicious_movement_pattern")
        elif behavior >= 0.4:
            reasons.append("stationary_or_repetitive_motion")
        if face > 0:
            reasons.append("face_visibility_low")
        if vlm >= 0.5:
            reasons.append("vlm_context_risk")
        return reasons

    @staticmethod
    def _clamp01(v) -> float:
        return max(0.0, min(1.0, float(v)))

    def assess_with_vlm(self, context) -> float:
        """Extension hook for scene-level VLM judgement.

        Keep returning 0.0 in the demo: no API key dependency, no latency risk.
        A future implementation can inspect context/frame metadata and return
        a normalized suspiciousness score in [0, 1].
        """
        return self._clamp01(context.get("vlm_score", 0.0))


if __name__ == "__main__":
    eng = SuspicionEngine(suspect_secs=2.5)
    seq = [True]*30 + [False]*10 + [True]*30
    t = 0.0
    for i, d in enumerate(seq):
        r = eng.update(d, 0.1); t += 0.1
        if r["fire"] or i % 10 == 0:
            print("t=%.1f det=%s level=%s seen=%.2f fire=%s" %
                  (t, d, r["level"], r["seen_time"], r["fire"]))

    ctx = SuspicionEngine(suspect_secs=4.0)
    sample = {"present": True, "zone": {"id": "datacenter_perimeter",
              "type": "sensitive_perimeter", "risk_weight": 0.55,
              "loitering_allowed": False}, "after_hours": True,
              "movement_pattern": "pace"}
    for _ in range(40):
        out = ctx.update(sample, 0.1)
    print("context level=%s score=%.2f reasons=%s" %
          (out["level"], out["score"], ",".join(out["reasons"])))
