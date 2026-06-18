"""
GIST X+AI Digital Twin — 순찰 로봇 기반 수상한 사람 탐지 (라이브 Isaac 시연용)
시나리오: 로봇이 지정 경로를 순찰 -> 시야/범위 안에 사람이 일정시간 머무르면 '수상자' 판정
          -> 로그 기록 + 3D 빨간 박스 표시 + 접근/확인 -> 복귀.

실행:
  라이브(GUI):   DEMO_HEADLESS=0 /home/user/Desktop/isaacsim/python.sh dt_demo.py
  헤드리스 테스트: DEMO_HEADLESS=1 DEMO_SECONDS=45 /home/user/Desktop/isaacsim/python.sh dt_demo.py

================== 사용자 편집 영역 ==================
좌표계: cm, (x, z). 걸어다닐 수 있는 영역 대략 X[0,2700] Z[0,3400].
"""
# --- 순찰 경로: 남쪽 사무동 + 북쪽 datacenter 접근까지 (navmesh가 통로로 자동 연결) ---
PATROL_ROUTE = [(1100, 2000), (1400, 2700), (1855, 3300), (1400, 3400)]  # 로봇0(카메라): 복도(중앙)에서 datacenter로 접근 -> 수상자 발각(빌드업)
# --- 사람들: (x, z, 행동[, route])  행동 = idle | pace | loop | wander | walk ---
#  수상자만 perimeter에서 정지/살핌(casing). 정상인은 각자 분리된 경로를 '어디서→어디로' 왕복 보행.
WALK_ROUTES = {
    "central": [(1500, 2600), (900, 2000), (1400, 2900)],   # public 중앙 복도
    "south":   [(1100, 1000), (700, 1200), (900, 1800)],    # 남쪽/사무 구역
    "west":    [(700, 2300), (300, 2700), (500, 1500)],     # 서쪽 통로
}
PEOPLE = [
    (1500, 2600, "walk", WALK_ROUTES["central"]),  # 정상 — 중앙 복도 보행
    (1100, 1000, "walk", WALK_ROUTES["south"]),    # 정상 — 남쪽 보행
    (700, 2300, "walk", WALK_ROUTES["west"]),      # 정상 — 서쪽 보행
]
# --- 탐지 파라미터 ---
SENSE_RANGE = 900.0      # 로봇 감지 최대거리(cm). 콘 길이와 동일(일관) -> 콘 안이면 평가
SENSE_FOV   = 40.0       # 감지 시야각(±도) — ±40=80°콘(실제 카메라 화각, 앞쪽만)
SUSPECT_SECS = 2.0       # 이 시간 이상 시야에 머물면 '수상자'
# =====================================================

import os, math, time, random, json
# 이 스크립트가 있는 폴더 기준 상대경로(zip 이관 시 어디에 풀든 동작). DEMO_BASE로 덮어쓰기 가능.
BASE = os.environ.get("DEMO_BASE") or os.path.dirname(os.path.abspath(__file__))
def P(*parts): return os.path.join(BASE, *parts)   # projects 내부 경로 헬퍼
HEADLESS = os.environ.get("DEMO_HEADLESS", "1") == "1"
USE_ANIMGRAPH_PEOPLE = os.environ.get("DEMO_ANIMGRAPH_PEOPLE", "1") == "1"
_last_state = [0.0]   # HUD 갱신 스로틀
RUN_SECONDS = float(os.environ.get("DEMO_SECONDS", "0"))
DEMO_CONE = os.environ.get("DEMO_CONE", "1") == "1"            # FOV 콘 표시(끄려면 DEMO_CONE=0)
CONE_RANGE = float(os.environ.get("DEMO_CONE_RANGE", "500"))   # 콘 시각 길이(cm). 짧을수록 벽통과/혼잡 적음
DOOR_NEAR = float(os.environ.get("DEMO_DOOR_NEAR", "1200"))    # R0가 담당문 이 거리내면 그 구역로봇은 안쪽 순찰
DEMO_COORD = os.environ.get("DEMO_COORD", "1") == "1"          # 동적 커버리지 분담(끄려면 DEMO_COORD=0)
REJUDGE_GAP = float(os.environ.get("DEMO_REJUDGE_GAP", "8"))   # clear된 사람을 이 초 이상 놓쳤다 재관측하면 재판단
MIN_ASSESS = float(os.environ.get("DEMO_MIN_ASSESS", "4"))     # 최소 이 초 관측(노랑 assessing) 후에야 suspicious 확정(자연스러운 판단 연출)

# ---------- 구역(zone) 메타데이터 + 위치->구역 판정 (zones.json) ----------
ZONES = {"zones": [], "exits": [], "demo_clock": {}}
try:
    with open(P("zones.json")) as _zf:
        ZONES = json.load(_zf)
except Exception as _ze:
    print(">>> zones.json 로드 실패(폴백 public):", _ze)
AFTER_HOURS = bool(ZONES.get("demo_clock", {}).get("after_hours", True))
EXITS_CFG = ZONES.get("exits", [])

def _in_bounds(x, z, b):
    return bool(b) and b["xmin"] <= x <= b["xmax"] and b["zmin"] <= z <= b["zmax"]

def _zone_by_id(zid):
    for zz in ZONES.get("zones", []):
        if zz.get("id") == zid: return zz
    return None

def zone_of(x, z):
    """위치 -> 구역 dict. 명시 박스(면적 작은=구체적 우선) 매칭, 없으면 public 폴백."""
    bounded = [zz for zz in ZONES.get("zones", []) if zz.get("bounds")]
    def _area(zz):
        b = zz["bounds"]; return (b["xmax"]-b["xmin"]) * (b["zmax"]-b["zmin"])
    for zz in sorted(bounded, key=_area):
        if _in_bounds(x, z, zz["bounds"]):
            return zz
    for zz in ZONES.get("zones", []):
        if zz.get("bounds") is None:
            return zz
    return {"id": "unknown", "type": "public_space", "access_level": "public",
            "loitering_allowed": True, "risk_weight": 0.1}

def dist_to_restricted(x, z):
    """datacenter_core 박스까지 최단 거리(cm). 없으면 None."""
    core = _zone_by_id("datacenter_core")
    b = core.get("bounds") if core else None
    if not b: return None
    dx = max(b["xmin"]-x, 0.0, x-b["xmax"])
    dz = max(b["zmin"]-z, 0.0, z-b["zmax"])
    return math.hypot(dx, dz)

def zone_signals(x, z, behavior):
    """suspicion.SuspicionEngine.update() 입력용 신호 dict (zone 맥락 포함)."""
    zz = zone_of(x, z)
    return {
        "zone_id": zz.get("id", ""),
        "zone_type": zz.get("type", ""),
        "access_level": zz.get("access_level", "public"),
        "loitering_allowed": zz.get("loitering_allowed", True),
        "loitering_threshold_sec": zz.get("loitering_threshold_sec", 4.0),
        "zone_risk": zz.get("risk_weight", 0.1),
        "restricted_location": zz.get("access_level") == "restricted",
        "near_restricted_area": zz.get("type") == "sensitive_perimeter",
        "distance_to_restricted_cm": dist_to_restricted(x, z),
        "after_hours": AFTER_HOURS,
        "movement_pattern": behavior,
    }

from isaacsim import SimulationApp
if HEADLESS:
    simulation_app = SimulationApp({"headless": True})
else:
    # 풀 Isaac Sim GUI experience -> 평소처럼 익숙한 뷰포트 조작/UI
    # Isaac 설치 경로의 experience kit. 설치 위치가 다르면 DEMO_ISAAC_EXP로 지정.
    _exp = os.environ.get("DEMO_ISAAC_EXP", "/home/user/Desktop/isaacsim/apps/isaacsim.exp.full.kit")
    simulation_app = SimulationApp({"headless": False, "experience": _exp})

import carb, omni, omni.usd
from isaacsim.core.utils.extensions import enable_extension
for e in ["omni.anim.navigation.bundle", "omni.anim.timeline", "omni.anim.graph.core"]:
    try: enable_extension(e); simulation_app.update()
    except Exception as ex: print("ext fail", e, ex)

USD_PATH = P("S7_navmesh_ready.usd")
WALK_USD = P("Walking.usd")
# omni.anim.people 캐릭터(이미 리깅+AnimationGraph: walk/idle/lookAround). 로컬 우선, 없으면 클라우드.
BIPED_SETUP_LOCAL = P("Biped_Setup.usd")
BIPED_SETUP_URL = "https://omniverse-content-production.s3.us-west-2.amazonaws.com/Assets/Isaac/4.5/Isaac/People/Characters/Biped_Setup.usd"
BIPED_SETUP = BIPED_SETUP_LOCAL if os.path.exists(BIPED_SETUP_LOCAL) else BIPED_SETUP_URL
HUSKY    = "/World/Ni_KI_Husky"
CAM_PATH = "/World/Ni_KI_Husky/front_bumper_link/husky_rgb_cam"
OUT_DIR  = P("demo_frames")
LOG_PATH = P("patrol_log.txt")
os.makedirs(OUT_DIR, exist_ok=True)

print(">>> opening stage ...")
omni.usd.get_context().open_stage(USD_PATH)
from isaacsim.core.utils.stage import is_stage_loading
simulation_app.update(); simulation_app.update()
while is_stage_loading(): simulation_app.update()
print(">>> stage loaded")

import numpy as np
from pxr import Gf, Usd, UsdGeom, UsdPhysics, PhysxSchema
import omni.timeline, omni.anim.navigation.core as navcore
try:
    import omni.anim.graph.core as ag
except Exception:
    ag = None
stage = omni.usd.get_context().get_stage()

# ---------- navmesh (World/물리 생성 전에 깨끗이 베이크 — 커버리지 보존) ----------
inav = navcore.acquire_interface()
try: inav.start_navmesh_baking_and_wait()
except Exception:
    inav.start_navmesh_baking()
    for _ in range(120): simulation_app.update()
for _ in range(60): simulation_app.update()
navmesh = inav.get_navmesh()
print(">>> navmesh ready")

def F3(x, y, z): return carb.Float3(float(x), float(y), float(z))
def snap(x, z, y=100.0):
    try:
        cp = navmesh.query_closest_point(F3(x, y, z))
        if cp is None: return F3(x, y, z)
        pt = cp[0] if isinstance(cp, (tuple, list)) else cp   # (point, flag) 반환 처리
        if hasattr(pt, "x"): return F3(pt.x, pt.y, pt.z)
        return F3(pt[0], pt[1], pt[2])
    except Exception: return F3(x, y, z)
def navpath(a, b, r=5.0):
    try:
        path = navmesh.query_shortest_path(a, b, agent_radius=r)
        if path is not None:
            pts = path.get_points()
            if pts and len(pts) >= 2:
                return [(float(p[0]), float(p[2])) for p in pts]
        print("  navpath FALLBACK (query None/empty)")
    except Exception as ex: print("  navpath err", ex)
    return [(float(a[0]), float(a[2])), (float(b[0]), float(b[2]))]

# 진단: navmesh가 벽을 아는가? (건물 가로지르는 먼 두 점)
_tA = F3(400, 100, 600); _tB = F3(2400, 100, 3000)
try:
    _tp = navmesh.query_shortest_path(_tA, _tB, agent_radius=5.0)
    if _tp is not None:
        _pp = _tp.get_points()
        _L = sum(math.hypot(_pp[i+1][0]-_pp[i][0], _pp[i+1][2]-_pp[i][2]) for i in range(len(_pp)-1))
        _straight = math.hypot(2400-400, 3000-600)
        print(">>> NAVMESH 진단: 가로지름 경로 %d pts, 길이 %.0f, 직선 %.0f, 배율 %.2f (벽인지=배율>1.3)"
              % (len(_pp), _L, _straight, _L/_straight))
    else:
        print(">>> NAVMESH 진단: query None")
except Exception as _ex:
    print(">>> NAVMESH 진단 err:", _ex)

# 순찰로 빌더: 점들을 navmesh 스냅 -> query_shortest_path -> 벽 피하는 굴곡 경로(다운샘플)
def build_route(points):
    raw = []
    for i in range(len(points)):
        a = snap(points[i][0], points[i][1])
        b = snap(points[(i+1) % len(points)][0], points[(i+1) % len(points)][1])
        seg = navpath(a, b, r=5.0)
        raw.extend(seg if not raw else seg[1:])
    if not raw:
        return [(float(x), float(z)) for (x, z) in points]
    out = [raw[0]]
    for p in raw[1:]:
        if math.hypot(p[0]-out[-1][0], p[1]-out[-1][1]) >= 40.0: out.append(p)
    if len(out) < 2:
        out = [(float(x), float(z)) for (x, z) in points]
    return out

ROUTE = build_route(PATROL_ROUTE)                                   # 로봇0(카메라): datacenter 경계
# 로봇1(E_MAIN 정문 담당) / 로봇2(E_EAST 서쪽문 담당): 평소엔 문 주변(door), R0가 그 문에 오면 안쪽(inner)
R1_DOOR  = build_route([(1250,400),(800,500),(500,900),(1000,650)])           # R1 문 주변(정문)
R1_INNER = build_route([(400,1500),(900,1700),(1100,2100),(500,1900)])        # R1 안쪽(남office 내부)
R2_DOOR  = build_route([(600,3600),(1000,3100),(1100,2500),(500,2900)])       # R2: 서쪽~복도 순회(캠핑X, 수상자 경로 근처 지나며 협동 판단)
R2_INNER = R2_DOOR
SUSPECT_ROUTE = [(1450,2900),(1712,3300)]                                     # 수상자: datacenter 근처서 짧게 잠입 -> 경계 도달 시 casing(접근 짧아 깜빡임↓)
SOUTH_ROUTE = R1_DOOR; MID_ROUTE = R2_DOOR                                    # 시작은 문 주변
print(">>> ROUTE pts: r0=%d south=%d mid=%d  r0범위 X[%.0f,%.0f] Z[%.0f,%.0f]" % (
    len(ROUTE), len(SOUTH_ROUTE), len(MID_ROUTE),
    min(p[0] for p in ROUTE), max(p[0] for p in ROUTE),
    min(p[1] for p in ROUTE), max(p[1] for p in ROUTE)))

# ---------- husky 키네마틱 + World (navmesh 베이크 후) ----------
def _disable_joints(root):
    """키네마틱 Husky의 물리 관절 비활성화 — PhysX 'CreateJoint between static bodies' 에러 제거(이동엔 무관)."""
    for q in Usd.PrimRange(stage.GetPrimAtPath(root)):
        if UsdPhysics.Joint(q):
            q.SetActive(False)
husky = stage.GetPrimAtPath(HUSKY)
try: PhysxSchema.PhysxArticulationAPI.Apply(husky).CreateArticulationEnabledAttr(False)
except Exception: pass
for q in Usd.PrimRange(husky):
    if UsdPhysics.RigidBodyAPI(q): UsdPhysics.RigidBodyAPI(q).CreateKinematicEnabledAttr(True)
_disable_joints(HUSKY)
from isaacsim.core.api import World
from isaacsim.sensors.camera import Camera
world = World(stage_units_in_meters=0.01); world.reset()
try: world.get_physics_context().set_gravity(0.0)
except Exception: pass

# ---------- 카메라 (GUI/headless 공통 탐지용) ----------
# 카메라 렌즈/높이 튜닝 (USD 직접) — 모드 무관
try:
    cu = UsdGeom.Camera(stage.GetPrimAtPath(CAM_PATH)); cu.GetFocalLengthAttr().Set(float(cu.GetFocalLengthAttr().Get())*0.5)
    cops = {op.GetOpName(): op for op in UsdGeom.Xformable(stage.GetPrimAtPath(CAM_PATH)).GetOrderedXformOps()}
    if "xformOp:translate" in cops:
        t = cops["xformOp:translate"].Get(); cops["xformOp:translate"].Set(Gf.Vec3d(t[0], t[1], t[2]+0.8))
except Exception as ex: print("cam tune skip:", ex)
# replicator render product + rgb annotator → GUI에서도 get_data()로 픽셀 획득
rgb_annot = None
try:
    import omni.replicator.core as rep
    _rp = rep.create.render_product(CAM_PATH, (640, 360))
    rgb_annot = rep.AnnotatorRegistry.get_annotator("rgb")
    rgb_annot.attach([_rp])
    print(">>> 카메라 render product 연결 (640x360)")
except Exception as ex:
    print("render product 생성 실패:", ex)

# ---------- 여러 사람 생성 (/World/Walking 자산 재사용) ----------
# 원본 /World/Walking 의 Y(바닥높이) 얻기
_orig = stage.GetPrimAtPath("/World/Walking")
PERSON_Y = 90.0
try:
    for op in UsdGeom.Xformable(_orig).GetOrderedXformOps():
        if op.GetOpName() == "xformOp:translate": PERSON_Y = float(op.Get()[1])
except Exception: pass
# 원본 숨김 (대신 우리가 만든 사람들 사용)
try: UsdGeom.Imageable(_orig).MakeInvisible()
except Exception: pass

_SPEED = {"idle": 0.0, "pace": 110.0, "loop": 120.0, "wander": 110.0, "walk": 140.0, "infiltrate": 230.0}  # infiltrate=빠르게 잠입(걷기 모션 유지)
WALK_BOB_CM = 3.0
WALK_SWAY_DEG = 4.0
class Person:
    def __init__(self, idx, sx, sz, behavior, route=None):
        self.id = "P%d" % (idx+1)
        self.behavior = behavior
        self.route = list(route) if route else None
        self.caught = False; self._latched = False
        self._case_pts = None
        self.path = "/World/Patrol_Person_%d" % idx
        self.char_root = "/World/human/human_%d" % idx
        self.anim_path = self.char_root + "/biped_demo_meters"   # Biped_Setup의 SkelRoot
        self.anim_char = None
        self.anim_ok = False
        self.anim_target = None
        self.anim_last = (float(sx), float(sz))
        self.anim_stuck_t = 0.0
        UsdGeom.Xform.Define(stage, self.path)
        if not (USE_ANIMGRAPH_PEOPLE and ag is not None) and os.path.exists(WALK_USD):  # 폴백 정적스킨(Walking.usd 있을 때만)
            ch = stage.DefinePrim(self.path + "/char", "Xform")
            ch.GetReferences().AddReference(WALK_USD)
        xf = UsdGeom.Xformable(stage.GetPrimAtPath(self.path)); xf.ClearXformOpOrder()
        self.t_op = xf.AddTranslateOp(); self.r_op = xf.AddRotateYOp()
        # omni.anim.people 캐릭터(Biped_Setup) 참조 — 미터단위 -> 100x(cm 씬)
        if USE_ANIMGRAPH_PEOPLE and ag is not None:
            try:
                cp = stage.DefinePrim(self.char_root, "Xform")
                cp.GetReferences().AddReference(BIPED_SETUP)
                cxf = UsdGeom.Xformable(cp); cxf.ClearXformOpOrder()
                cxf.AddTranslateOp().Set(Gf.Vec3d(float(sx), 0.0, float(sz)))
                cxf.AddScaleOp().Set(Gf.Vec3f(100.0, 100.0, 100.0))
            except Exception as ex:
                print(">>> biped ref fail", self.id, ex)
        self.bx, self.bz = float(sx), float(sz)   # 집(기준점)
        self.x, self.z = float(sx), float(sz)
        self.speed = _SPEED.get(behavior, 60.0)
        self.li = 0; self.phase = 0.0; self.motion_t = 0.0; self.yaw = 0.0
        # 패턴별 경유점
        if route:                self.pts = [tuple(p) for p in route]   # walk: 지정 경로
        elif behavior == "pace": self.pts = [(sx-180, sz), (sx+180, sz)]
        elif behavior == "loop": self.pts = [(sx-150, sz-150), (sx+150, sz-150), (sx+150, sz+150), (sx-150, sz+150)]
        else:                    self.pts = [(sx, sz)]
        self.target = (sx, sz)
        self.seen = 0.0; self.suspicious = False; self.logged = False
        self.vlm_score = 0.0; self.vlm_reason = ""; self.vlm_state = "none"   # VLM 판단 입력
        self._update_prim()
    def _update_prim(self, moving=False):
        bob = WALK_BOB_CM * abs(math.sin(self.motion_t * math.tau)) if moving else 0.0
        sway = WALK_SWAY_DEG * math.sin(self.motion_t * math.tau) if moving else 0.0
        self.t_op.Set(Gf.Vec3d(self.x, PERSON_Y + bob, self.z))
        self.r_op.Set(float(self.yaw + sway))
    def pos(self): return (self.x, self.z)
    def _sep(self):
        """다른 사람으로부터 분리(척력) 벡터 — 사람끼리 겹침 방지."""
        rx = rz = 0.0
        for o in people:
            if o is self: continue
            ox, oz = o.x - self.x, o.z - self.z
            od = math.hypot(ox, oz)
            if 1.0 < od < 150.0:
                w = (150.0 - od) / 150.0
                rx -= ox/od*w; rz -= oz/od*w
        return rx*1.6, rz*1.6
    def reconfigure(self, sx, sz, behavior, route=None):
        """시나리오 전환: 위치/행동/판단상태 리셋."""
        self.behavior = behavior
        self.route = list(route) if route else None
        self.caught = False; self._latched = False; self._assess_t = 0.0; self._rerouted = False
        self._case_pts = None
        self.bx, self.bz = float(sx), float(sz)
        self.x, self.z = float(sx), float(sz)
        self.speed = _SPEED.get(behavior, 60.0)
        self.li = 0; self.phase = 0.0; self.motion_t = 0.0; self.yaw = 0.0
        if route:                self.pts = [tuple(p) for p in route]
        elif behavior == "pace": self.pts = [(sx-180, sz), (sx+180, sz)]
        elif behavior == "loop": self.pts = [(sx-150, sz-150), (sx+150, sz-150), (sx+150, sz+150), (sx-150, sz+150)]
        else:                    self.pts = [(sx, sz)]
        self.target = (sx, sz)
        self.seen = 0.0; self.suspicious = False
        self.vlm_score = 0.0; self.vlm_reason = ""; self.vlm_state = "none"
        self._susp_prev = False; self._nav_goal = None; self.flee_wp = None; self.flee_i = 1
        if self.anim_ok:
            self._set_anim_idle(); self._set_anim_transform(self.x, self.z, self.yaw); self._sync_from_anim()
        else:
            self._update_prim()
    def init_animgraph(self):
        def _dbg(msg):
            try: open("/tmp/anim_dbg.txt", "a").write("%s %s: %s\n" % (self.id, self.anim_path, msg))
            except Exception: pass
        if not (USE_ANIMGRAPH_PEOPLE and ag is not None and self.anim_path):
            _dbg("skip(flag/ag/path)"); return False
        prim = stage.GetPrimAtPath(self.anim_path)
        if not prim.IsValid():
            _dbg("prim INVALID"); return False
        try:
            self.anim_char = ag.get_character(self.anim_path)
            if self.anim_char is None:
                _dbg("get_character None"); return False
            _dbg("OK")
            self.anim_ok = True
            UsdGeom.Imageable(stage.GetPrimAtPath(self.path)).MakeInvisible()
            self._set_anim_transform(self.x, self.z, self.yaw)
            self._set_anim_idle()
            print(">>> AnimGraph person", self.id, "->", self.anim_path)
            return True
        except Exception as ex:
            print(">>> AnimGraph init fail", self.id, self.anim_path, ex)
            self.anim_char = None; self.anim_ok = False
            return False
    def _yaw_quat(self, yaw_deg):
        h = math.radians(yaw_deg) * 0.5
        return carb.Float4(0.0, math.sin(h), 0.0, math.cos(h))
    def _set_anim_transform(self, x, z, yaw_deg):
        try:
            self.anim_char.set_world_transform(F3(x, PERSON_Y, z), self._yaw_quat(yaw_deg))
        except Exception:
            pass
    def _set_anim_idle(self, look=False):
        if not self.anim_ok: return
        try:
            self.anim_char.set_variable("Action", "None")
            self.anim_char.set_variable("Walk", 0.0)
            self.anim_char.set_variable("PathPoints", [])
            self.anim_char.set_variable("lookAround", 1.0 if look else 0.0)   # 두리번(casing)
        except Exception:
            pass
    def _sync_from_anim(self):
        if not self.anim_ok: return
        try:
            pos = carb.Float3(0.0, 0.0, 0.0)
            rot = carb.Float4(0.0, 0.0, 0.0, 1.0)
            self.anim_char.get_world_transform(pos, rot)
            self.x, self.z = float(pos.x), float(pos.z)
        except Exception:
            pass
    def _drive_anim_to(self, tx, tz, dt):
        if not self.anim_ok:
            return False
        dx, dz = tx - self.x, tz - self.z
        d = math.hypot(dx, dz)
        if d < 45.0:
            self._set_anim_idle()
            self.anim_target = None
            return True
        self.yaw = math.degrees(math.atan2(dx, dz))
        start = snap(self.x, self.z)
        goal = snap(tx, tz)
        pts = navpath(start, goal, r=5.0)
        path_points = [F3(px, PERSON_Y, pz) for px, pz in pts[:80]]
        if len(path_points) < 2:
            path_points = [F3(self.x, PERSON_Y, self.z), F3(tx, PERSON_Y, tz)]
        try:
            self.anim_char.set_variable("Action", "Walk")
            self.anim_char.set_variable("PathPoints", path_points)
            self.anim_char.set_variable("Walk", 1.0)
            self.anim_char.set_variable("lookAround", 0.0)   # 걸을 땐 두리번 OFF(이전 casing 값 잔존 방지)
        except Exception as ex:
            print(">>> AnimGraph drive fail", self.id, ex)
            self.anim_ok = False
            UsdGeom.Imageable(stage.GetPrimAtPath(self.path)).MakeVisible()
            return False
        last_x, last_z = self.anim_last
        moved = math.hypot(self.x - last_x, self.z - last_z)
        self.anim_stuck_t = self.anim_stuck_t + dt if moved < 0.5 else 0.0
        self.anim_last = (self.x, self.z)
        if self.anim_stuck_t > 2.0:
            print(">>> AnimGraph stuck -> fallback", self.id)
            self.anim_ok = False
            UsdGeom.Imageable(stage.GetPrimAtPath(self.path)).MakeVisible()
            return False
        self._sync_from_anim()
        return False
    def step(self, dt):
        if self.anim_ok and self.behavior != "flee":
            self._sync_from_anim()
        if self.behavior == "flee":
            # 가까운 출구가 막혔으면 -> 반대 출구로 1회 재도주(막혀 있어도 거기로 가서 검거 = 추격 드라마)
            if _exit_blocked(self.flee_target) and not getattr(self, "_rerouted", False):
                _others = [(float(e["x"]), float(e["z"])) for e in EXITS_CFG
                           if math.hypot(float(e["x"]) - self.flee_target[0], float(e["z"]) - self.flee_target[1]) > 1.0]
                if _others:
                    alt = _others[0]                                       # 반대 출구
                    self.flee_target = alt; self._rerouted = True
                    self.flee_wp = navpath(snap(self.x, self.z), snap(alt[0], alt[1]), r=5.0); self.flee_i = 1
                    log_event(">>> REROUTE %s -> 반대 출구 (%.0f,%.0f)" % (self.id, alt[0], alt[1]))
            # 출구를 막은 로봇 앞에서 정지(궁지에 몰림) — 통과/핑퐁 방지
            blocker = _exit_robot(self.flee_target)
            if blocker is not None and math.hypot(blocker.x-self.x, blocker.z-self.z) < 240.0:
                if not self.caught:
                    log_event(">>> CORNERED %s at exit by %s" % (self.id, getattr(blocker, "id", "robot")))
                self.caught = True
                self.yaw = math.degrees(math.atan2(blocker.x-self.x, blocker.z-self.z))  # 로봇 응시
                if self.anim_ok:
                    self._set_anim_idle(look=False); self._set_anim_transform(self.x, self.z, self.yaw)
                else:
                    self._update_prim()
                return
            wp = getattr(self, "flee_wp", None)
            if wp and len(wp) >= 2:
                while self.flee_i < len(wp)-1 and math.hypot(wp[self.flee_i][0]-self.x, wp[self.flee_i][1]-self.z) < 45.0:
                    self.flee_i += 1
                tx, tz = wp[min(self.flee_i, len(wp)-1)]
            else:
                tx, tz = self.flee_target
            dx, dz = tx - self.x, tz - self.z; d = math.hypot(dx, dz) or 1.0
            sx_, sz_ = self._sep()                               # 다른 사람 회피(분리)
            gx, gz = dx/d + sx_, dz/d + sz_; gd = math.hypot(gx, gz) or 1.0
            self.x += gx/gd*FLEE_SPEED*dt; self.z += gz/gd*FLEE_SPEED*dt   # 빠른 도주
            if sx_ or sz_:                                                 # 분리로 밀렸을 때만 walkable 스냅
                _sp = snap(self.x, self.z); self.x, self.z = _sp.x, _sp.z
            self.yaw = math.degrees(math.atan2(gx, gz)); self.motion_t += dt*1.8
            if self.anim_ok:
                try:
                    self.anim_char.set_variable("Action", "Walk")
                    self.anim_char.set_variable("Walk", 1.0)
                    self.anim_char.set_variable("PathPoints", [])    # 제자리 걷기(root motion 끔) -> 텔레포트로 이동
                    self.anim_char.set_variable("lookAround", 0.0)   # 도주 중엔 두리번 OFF
                except Exception: pass
                self._set_anim_transform(self.x, self.z, self.yaw)   # 논리위치로 텔레포트 → 박스와 일치
            else:
                self._update_prim(moving=True)
            return
        if self.behavior == "infiltrate" and zone_of(self.x, self.z).get("type") in ("sensitive_perimeter", "restricted_area"):
            self.behavior = "idle"; self.bx, self.bz = self.x, self.z; self.phase = 0.0   # datacenter 앞(도달가능 경계) 도착 -> casing -> 발각
        if self.behavior == "idle":   # 제자리 casing: 한 자리에서 크게 두리번거리며 주변을 살핌
            self.x, self.z = self.bx, self.bz                  # 위치 고정(이동 X)
            self.phase += dt
            # 큰 몸 회전(±135°) + 작은 빠른 흔들림(±25°) 합성 -> 적극적으로 사방을 살피는 모습
            self.yaw = 135.0 * math.sin(self.phase * 0.45) + 25.0 * math.sin(self.phase * 1.9)
            if self.anim_ok:
                self._set_anim_idle(look=True)                 # lookAround 애니 ON
                self._set_anim_transform(self.x, self.z, self.yaw)
            else:
                self._update_prim()
            return
        if self.behavior == "wander":
            tx, tz = self.target
            if math.hypot(tx - self.x, tz - self.z) < 45.0:
                self.target = (self.bx + random.uniform(-280, 280), self.bz + random.uniform(-280, 280))
                tx, tz = self.target
        else:  # pace / loop / walk (지정 경로 순회)
            if getattr(self, "_pause_t", 0.0) > 0.0:          # 경유점에서 잠깐 멈춤(자연스러운 보행)
                self._pause_t -= dt
                self.phase += dt; self.yaw = 25.0 * math.sin(self.phase * 0.6)
                if self.anim_ok:
                    self._set_anim_idle(); self._set_anim_transform(self.x, self.z, self.yaw)
                else:
                    self._update_prim()
                return
            tx, tz = self.pts[self.li]
            if math.hypot(tx - self.x, tz - self.z) < 60.0:
                if self.behavior == "infiltrate" and self.li >= len(self.pts)-1:   # datacenter 앞 도달 -> casing
                    self.behavior = "idle"; self.bx, self.bz = self.x, self.z; self.phase = 0.0
                    return
                self.li = (self.li + 1) % len(self.pts)
                self._pause_t = 0.0 if self.behavior == "infiltrate" else random.uniform(0.5, 1.6)  # 잠입은 안 멈추고 직진
                return
        # navmesh waypoint로 '수동' 이동(속도 제어 가능) + 사람끼리 회피. anim은 제자리걷기로 다리만 움직임.
        nx, nz = self._nav_step(tx, tz)
        dx, dz = nx - self.x, nz - self.z; d = math.hypot(dx, dz) or 1.0
        sx_, sz_ = self._sep()
        gx, gz = dx/d + sx_, dz/d + sz_; gd = math.hypot(gx, gz) or 1.0
        self.x += gx/gd*self.speed*dt; self.z += gz/gd*self.speed*dt
        if sx_ or sz_:                                                     # 분리로 밀렸을 때만 walkable 스냅
            _sp = snap(self.x, self.z); self.x, self.z = _sp.x, _sp.z
        self.motion_t += dt * max(0.8, self.speed / 70.0)
        self.yaw = math.degrees(math.atan2(gx, gz))
        if self.anim_ok:
            try:
                self.anim_char.set_variable("Action", "Walk")
                self.anim_char.set_variable("Walk", 1.0)
                self.anim_char.set_variable("PathPoints", [])     # 제자리걷기(root motion 끔) -> 텔레포트로 이동
                self.anim_char.set_variable("lookAround", 0.0)
            except Exception: pass
            self._set_anim_transform(self.x, self.z, self.yaw)
        else:
            self._update_prim(moving=True)
    def _nav_step(self, tx, tz):
        g = (round(tx/20.0), round(tz/20.0))
        if getattr(self, "_nav_goal", None) != g:
            self._nav_goal = g
            self._nav_wp = navpath(snap(self.x, self.z), snap(tx, tz), r=5.0)
            self._nav_i = 1
        wp = getattr(self, "_nav_wp", None)
        if not wp or len(wp) < 2:
            return (tx, tz)
        while self._nav_i < len(wp)-1 and math.hypot(wp[self._nav_i][0]-self.x, wp[self._nav_i][1]-self.z) < 45.0:
            self._nav_i += 1
        return wp[min(self._nav_i, len(wp)-1)]

people = [Person(i, p[0], p[1], p[2], p[3] if len(p) > 3 else None) for i, p in enumerate(PEOPLE)]
print(">>> 사람 수:", len(people), [(p.id, p.behavior) for p in people])

# ---------- husky 주행 ----------
hops = {op.GetOpName(): op for op in UsdGeom.Xformable(husky).GetOrderedXformOps()}
t_op, o_op = hops["xformOp:translate"], hops["xformOp:orient"]
R0 = Gf.Quatd(-0.001104476819168599, 0.0016000063325766981, -0.7239642162643103, -0.6898347872349542)
HEADING_OFFSET = math.pi/2
FIXED_Y = float(t_op.Get()[1])
PATROL_SPEED, INTERCEPT_SPEED = 200.0, 600.0   # 순찰=느림, 차단=빠름(도주260보다 충분히 빨라 확실히 선점)
PURSUE_SPEED = 300.0                           # 추격=도주(260)보다 살짝만 빠르게 -> 자연스러운 추격(순간이동 X)
TURN_RATE = math.radians(200); ARRIVE = 130.0; REACH = 90.0
OBSERVE_SEC = 3.0   # 수상자 감지 시 제자리 관측 시간(초)

def wrap(a): return (a + math.pi) % (2*math.pi) - math.pi
def _mk_set_orient(o):
    def _set(yaw):
        h = yaw*0.5; o.Set(Gf.Quatd(math.cos(h), 0.0, math.sin(h), 0.0) * R0)
    return _set

class Robot:
    def __init__(self, rid, t_op, o_op, route, start, role="patrol", has_cam=False):
        self.id = rid; self.t_op = t_op; self.o_op = o_op
        self.set_orient = _mk_set_orient(o_op)
        self.route = route; self.role = role; self.has_cam = has_cam
        self.x, self.z = float(start[0]), float(start[1])
        self.start_xz = (float(start[0]), float(start[1]))   # 시나리오 리셋용
        self.phi = 0.0; self.pi = 1
        self.mode = "PATROL" if role == "patrol" else "IDLE"
        self.observe_t = 0.0; self.observe_id = None
        self.goal = None; self.intercept_exit = None; self.pursue_target = None
        self.goal_path = []; self.gpi = 1; self._repath_t = 0.0   # navmesh 경로 추종
        self._apply()
    def _apply(self):
        self.t_op.Set(Gf.Vec3d(self.x, FIXED_Y, self.z)); self.set_orient(self.phi + HEADING_OFFSET)
    def drive(self, tx, tz, speed, dt):
        gtx, gtz = tx, tz
        # 사람 회피(동적 장애물): 근처 사람으로부터 척력 -> 목표방향과 합성해 비껴감
        rx = rz = 0.0
        for p in people:
            if p is self.pursue_target: continue    # 추격 대상은 회피 안 함
            pdx, pdz = self.x - p.x, self.z - p.z
            pd = math.hypot(pdx, pdz)
            if 1.0 < pd < 160.0:
                w = (160.0 - pd) / 160.0
                rx += pdx/pd * w; rz += pdz/pd * w
        _yield = 1.0
        _myi = robots.index(self) if self in robots else 0
        for orb in robots:                      # 로봇끼리 회피(겹침 방지) — 부드럽게 + 우선순위 양보
            if orb is self: continue
            odx, odz = self.x - orb.x, self.z - orb.z
            od = math.hypot(odx, odz)
            if 1.0 < od < 180.0:
                w = (180.0 - od) / 180.0
                rx += odx/od * w * 0.7; rz += odz/od * w * 0.7   # 척력 약화(진동↓)
            # 양보: 가까우면 뒤 인덱스 로봇이 감속해 비켜줌(둘 다 휘청대는 대칭 깨기)
            if od < 160.0 and _myi > (robots.index(orb) if orb in robots else 0):
                _yield = min(_yield, 0.25)
        if rx or rz:
            tdx, tdz = tx - self.x, tz - self.z; td = math.hypot(tdx, tdz) or 1.0
            gx, gz = tdx/td + rx*1.0, tdz/td + rz*1.0          # 합성 약화(목표방향 우선)
            gtx, gtz = self.x + gx*120.0, self.z + gz*120.0
        dx, dz = gtx - self.x, gtz - self.z
        err = wrap(math.atan2(dx, dz) - self.phi)
        self.phi += max(-TURN_RATE*dt, min(TURN_RATE*dt, err))
        if abs(err) < math.radians(35):
            self.x += math.sin(self.phi)*speed*_yield*dt
            self.z += math.cos(self.phi)*speed*_yield*dt
            if rx or rz:   # 회피로 밀렸을 때만 walkable로 스냅(정상 순찰 이동은 방해 X, 벽 통과만 방지)
                _sp = snap(self.x, self.z); self.x, self.z = _sp.x, _sp.z
        self._apply()
        return math.hypot(tx - self.x, tz - self.z)   # 실제 목표까지 거리(보정 전)
    def set_nav_goal(self, target):
        self.goal = (float(target[0]), float(target[1]))
        try:
            self.goal_path = navpath(snap(self.x, self.z), snap(self.goal[0], self.goal[1]), r=5.0)
        except Exception:
            self.goal_path = [(self.x, self.z), self.goal]
        self.gpi = 1
    def _follow_path(self, speed, dt):
        wp = self.goal_path
        if not wp or len(wp) < 2:
            return self.drive(self.goal[0], self.goal[1], speed, dt)
        while self.gpi < len(wp)-1 and math.hypot(wp[self.gpi][0]-self.x, wp[self.gpi][1]-self.z) < REACH:
            self.gpi += 1
        tx, tz = wp[min(self.gpi, len(wp)-1)]
        self.drive(tx, tz, speed, dt)
        return math.hypot(self.goal[0]-self.x, self.goal[1]-self.z)
    def step(self, dt, observed):
        if self.mode == "PURSUE" and self.pursue_target is not None:
            pd = math.hypot(self.pursue_target.x - self.x, self.pursue_target.z - self.z)
            if pd < 160.0:                                # 근접 -> 정지 후 응시(겹침·빙글빙글 방지)
                self.set_orient(math.atan2(self.pursue_target.x - self.x,
                                           self.pursue_target.z - self.z) + HEADING_OFFSET)
                return
            self._repath_t -= dt                          # 도주자 위치로 주기적 재경로(navmesh)
            if self._repath_t <= 0 or not self.goal_path:
                self._repath_t = 0.4
                self.set_nav_goal((self.pursue_target.x, self.pursue_target.z))
            self._follow_path(PURSUE_SPEED, dt)   # 추격은 사람 속도에 맞춤(자연스러움); 출구 차단만 INTERCEPT_SPEED
            return
        if self.mode == "INTERCEPT":
            if self._follow_path(INTERCEPT_SPEED, dt) < ARRIVE:   # navmesh 경로로 출구 이동
                self.mode = "BLOCK"          # 출구 도착 -> 봉쇄 정지
        elif self.mode == "BLOCK" or self.mode == "IDLE":
            return                           # 정지 대기
        elif self.mode == "PATROL":
            # 동적 분담: R0가 내 담당 문에 가까우면 안쪽(inner) 순찰, 아니면 문 주변(door) 순찰
            if DEMO_COORD and getattr(self, "watch_door", None) is not None:
                _d = math.hypot(robots[0].x - self.watch_door[0], robots[0].z - self.watch_door[1])
                _inner_now = (self.route is self.route_inner)   # 히스테리시스: 들어갈 땐<DOOR_NEAR, 나올 땐>1.4배
                if not _inner_now and _d < DOOR_NEAR:      _want = self.route_inner
                elif _inner_now and _d > DOOR_NEAR * 1.4:  _want = self.route_door
                else:                                      _want = self.route
                if _want is not self.route:                # 전환 시 가장 가까운 waypoint + navmesh 재경로(직선점프 방지)
                    self.route = _want
                    self.pi = min(range(len(_want)), key=lambda i: math.hypot(_want[i][0]-self.x, _want[i][1]-self.z))
                    self.goal_path = None
            r = self.route
            while self.pi < len(r)-1 and math.hypot(r[self.pi][0]-self.x, r[self.pi][1]-self.z) < REACH:
                self.pi += 1
            tgt = r[self.pi]
            # 항상 navmesh로 이동(벽 우회). 목표 waypoint 바뀌거나 주기마다 재경로.
            self._repath_t -= dt
            if (not self.goal_path) or self.goal is None or self._repath_t <= 0 \
                    or math.hypot(self.goal[0]-tgt[0], self.goal[1]-tgt[1]) > REACH:
                self._repath_t = 0.5
                self.set_nav_goal(tgt)
            if self._follow_path(PATROL_SPEED, dt) < ARRIVE and self.pi >= len(r)-1:
                self.pi = 0
        elif self.mode == "OBSERVE":
            self.observe_t -= dt
            if observed is not None:
                px, pz = observed.pos()
                od = math.hypot(px - self.x, pz - self.z)
                if od > 260.0:                       # 멀어지면 navmesh 경로로 따라가며 관찰(벽 통과 X)
                    self._repath_t -= dt
                    if self._repath_t <= 0 or not self.goal_path:
                        self._repath_t = 0.4
                        self.set_nav_goal((px, pz))
                    self._follow_path(PATROL_SPEED, dt)
                else:                                # 가까우면 멈춰서 응시 (phi도 갱신 -> 파란 콘이 대상 따라 회전)
                    self.phi = math.atan2(px - self.x, pz - self.z)
                    self.set_orient(self.phi + HEADING_OFFSET)
                # 민감구역(수상자)이면 발각될 때까지 계속 관찰. 공개구역(정상인)은 VLM 판정 끝나면 통과.
                _sz = zone_of(px, pz).get("type") in ("sensitive_perimeter", "restricted_area")
                if _sz or getattr(observed, "vlm_state", "done") != "done":
                    self.observe_t = max(self.observe_t, 1.0)
            if self.observe_t <= 0:
                self.mode = "PATROL"; self.observe_id = None
    def observe(self, person):
        if self.mode in ("INTERCEPT", "BLOCK"): return
        self.mode = "OBSERVE"; self.observe_t = OBSERVE_SEC; self.observe_id = person.id
    def go_intercept(self, exit_id, target):
        self.intercept_exit = exit_id; self.set_nav_goal(target); self.mode = "INTERCEPT"
    def go_pursue(self, person):
        self.pursue_target = person; self.observe_id = person.id; self._repath_t = 0.0
        self.set_nav_goal((person.x, person.z)); self.mode = "PURSUE"

# ---------- Husky 3대: 원본 + 복제 2대 ----------
from pxr import Sdf
def _xops(path):
    h = {op.GetOpName(): op for op in UsdGeom.Xformable(stage.GetPrimAtPath(path)).GetOrderedXformOps()}
    return h["xformOp:translate"], h["xformOp:orient"]
def _clone_husky(idx):
    dst = "/World/Ni_KI_Husky_%d" % idx
    if not stage.GetPrimAtPath(dst).IsValid():
        Sdf.CopySpec(stage.GetRootLayer(), Sdf.Path(HUSKY), stage.GetRootLayer(), Sdf.Path(dst))
    for q in Usd.PrimRange(stage.GetPrimAtPath(dst)):
        rb = UsdPhysics.RigidBodyAPI(q)
        if rb: rb.CreateKinematicEnabledAttr(True)
    _disable_joints(dst)
    return dst

# 출구 좌표(zones.json) — 로봇1·2 순찰 시작점
def _exit_xz(eid, default):
    for e in EXITS_CFG:
        if e.get("id") == eid: return (float(e["x"]), float(e["z"]))
    return default
E_MAIN_XZ = _exit_xz("E_MAIN", (1250, 100))
E_EAST_XZ = _exit_xz("E_EAST", (600, 3600))
# 로봇0: 카메라 순찰(datacenter). 로봇1: E_MAIN에서 시작, 로봇2: E_EAST에서 시작.
ROBOT_DEFS = [(HUSKY, ROUTE, (ROUTE[0][0], ROUTE[0][1]), "patrol", True)]
try:
    p2 = _clone_husky(2); p3 = _clone_husky(3)
    ROBOT_DEFS.append((p2, SOUTH_ROUTE, E_MAIN_XZ, "patrol", False))   # 정문에서 순찰 시작
    ROBOT_DEFS.append((p3, MID_ROUTE, E_EAST_XZ, "patrol", False))     # 서쪽문에서 순찰 시작
    print(">>> Husky 복제 2대 완료")
except Exception as ex:
    print(">>> Husky 복제 실패(단일 로봇):", ex)

robots = []
for path, route, start, role, has_cam in ROBOT_DEFS:
    t_, o_ = _xops(path)
    robots.append(Robot(path.split("/")[-1], t_, o_, route, start, role, has_cam))
robot = robots[0]   # 카메라/POV 기준(하위 호환)
# 동적 분담: R1=정문(E_MAIN)/R2=서쪽문(E_EAST) 담당. R0가 그 문에 오면 inner, 아니면 door.
if len(robots) >= 3:
    robots[1].watch_door = E_MAIN_XZ; robots[1].route_door = R1_DOOR; robots[1].route_inner = R1_INNER
    robots[2].watch_door = E_EAST_XZ; robots[2].route_door = R2_DOOR; robots[2].route_inner = R2_INNER
try:
    open("/tmp/robots.txt", "w").write("robots=%d ids=%s\n" % (len(robots), [r.id for r in robots]))
except Exception: pass

# ---- 멀티 카메라: 각 로봇 카메라에 render product (라운드로빈 탐지) ----
DEMO_MULTICAM = os.environ.get("DEMO_MULTICAM", "1") == "1"   # 3카메라(공유 ownership으로 중복판단 제거). 1대만 쓰려면 DEMO_MULTICAM=0
try: import omni.replicator.core as rep
except Exception: rep = None
robot_cams = []
if rgb_annot is not None:
    robot_cams.append((robots[0], rgb_annot))           # 로봇0(POV/증거 기준)
if DEMO_MULTICAM and rep is not None:
    for rb in robots[1:]:
        try:
            cpath = "/World/%s/front_bumper_link/husky_rgb_cam" % rb.id
            _rp2 = rep.create.render_product(cpath, (320, 180))   # 3대 모드: 저해상도(렌더 부하↓)
            _an2 = rep.AnnotatorRegistry.get_annotator("rgb"); _an2.attach([_rp2])
            robot_cams.append((rb, _an2))
        except Exception as ex: print("멀티카메라 실패", rb.id, ex)
    print(">>> 카메라 대수:", len(robot_cams), "(MULTICAM, 320x180)")
if not robot_cams and rgb_annot is not None:
    robot_cams = [(robots[0], rgb_annot)]
try:   # 진단: 프로세스가 실제로 본 설정을 파일로 남김(백그라운드 stdout 유실 대비)
    with open("/tmp/dt_config.txt", "w") as _cf:
        _cf.write("cams=%d MULTICAM=%s HEADLESS=%s RUN_SECONDS=%s VLM_MODEL=%s VLM_SUSPECT=%s DEMO_ONLY=%r\n" % (
            len(robot_cams), DEMO_MULTICAM, HEADLESS, RUN_SECONDS, VLM_MODEL, VLM_SUSPECT,
            os.environ.get("DEMO_ONLY", "")))
except Exception: pass

tl = omni.timeline.get_timeline_interface(); tl.set_looping(True); tl.play()
for _ in range(10): world.step(render=False)
# 클라우드 캐릭터 로딩 + anim graph가 캐릭터 등록하도록 충분히 대기(ag.get_character는 runtime 전용)
try:
    from isaacsim.core.utils.stage import is_stage_loading as _isl
    for _ in range(120):
        simulation_app.update()
        if not _isl(): break
except Exception: pass
for _ in range(120): world.step(render=False)
# 초기화 재시도(등록 타이밍 대비)
animgraph_count = 0
for _try in range(25):
    animgraph_count = sum(1 for p in people if (p.anim_ok or p.init_animgraph()))
    if animgraph_count >= len(people): break
    for _ in range(15): world.step(render=False)
if USE_ANIMGRAPH_PEOPLE:
    print(">>> AnimGraph walkers:", animgraph_count, "/", len(people))
try:
    open("/tmp/anim.txt", "w").write("animgraph_walkers=%d/%d\n" % (animgraph_count, len(people)))
except Exception: pass

# GUI 시작 카메라(부감) + 3D 박스
DRAW = None; HUD = None; LABEL3D = {}
def _oui_color(r, g, b):
    try:
        import omni.ui as _u; return _u.color(r, g, b)
    except Exception: return 0xFFFFFFFF
if not HEADLESS:
    for _ in range(5): world.step(render=True)   # persp 카메라 준비 대기
    # 궤도 회전중심(COI)을 '건물 실제 중앙'으로 -> 평소 USD 열 때처럼 자연스러운 조작
    try:
        from isaacsim.util.debug_draw import _debug_draw
        DRAW = _debug_draw.acquire_debug_draw_interface()
    except Exception as ex: print("debug_draw skip:", ex)
    # in-Isaac 운영자 HUD (이벤트 로그 + 경보) — 웹 대시보드 대체
    try:
        import omni.ui as ui
        _hud_win = ui.Window("DT OPERATOR", width=520, height=620)
        with _hud_win.frame:
            with ui.VStack(spacing=8):
                _hud_status = ui.Label("t=0", height=40, word_wrap=True, style={"font_size": 26, "color": 0xFF66FF66})
                ui.Line(height=2)
                ui.Label("EVENT LOG", height=26, style={"font_size": 18, "color": 0xFF9FB0C4})
                _hud_events = ui.Label("(no events)", word_wrap=True, alignment=ui.Alignment.LEFT_TOP,
                                       style={"font_size": 17, "color": 0xFFE6EAF0})
        HUD = (_hud_status, _hud_events)
        print(">>> omni.ui HUD 패널 생성")
    except Exception as ex: print("omni.ui HUD skip:", ex)
    # 카메라 POV 보조 뷰포트: 모든 로봇 카메라(좌측). DEMO_POV=0 끄기, DEMO_POV_ALL=0 으로 로봇0만.
    POV_WINS = []
    if os.environ.get("DEMO_POV", "1") == "1":
        try:
            from omni.kit.viewport.utility import create_viewport_window
            _cams = [("Husky POV", CAM_PATH)]
            if os.environ.get("DEMO_POV_ALL", "1") == "1":
                for rb in robots[1:]:
                    _cams.append(("%s POV" % rb.id, "/World/%s/front_bumper_link/husky_rgb_cam" % rb.id))
            for nm, cp in _cams:
                w = create_viewport_window(nm, width=340, height=200)
                try: w.viewport_api.set_active_camera(cp)
                except Exception as ex: print("POV cam skip", nm, ex)
                POV_WINS.append(nm)
            print(">>> POV 뷰포트:", POV_WINS)
        except Exception as ex: print("POV viewport skip:", ex)
    # 좌측에 POV들 세로로 쌓고 맨 아래 HUD 도킹
    try:
        import omni.ui as ui
        for _ in range(8): world.step(render=True)        # 창 실체화 대기
        vpw = ui.Workspace.get_window("Viewport")
        hw  = ui.Workspace.get_window("DT OPERATOR")
        items = list(POV_WINS) + (["DT OPERATOR"] if hw else [])   # 좌측 컬럼: POV들 + HUD(가중 분할)
        weights = [1.0]*len(POV_WINS) + ([2.0] if hw else [])      # HUD를 POV의 2배 크기로
        suffix = [sum(weights[i:]) for i in range(len(weights))] + [1.0]
        anchor = vpw
        for i, nm in enumerate(items):
            w = ui.Workspace.get_window(nm)
            if not (w and anchor): continue
            if anchor is vpw:
                w.dock_in(vpw, ui.DockPosition.LEFT, 0.28)          # 좌측 컬럼 28% 폭(조금 넓게)
            else:
                w.dock_in(anchor, ui.DockPosition.BOTTOM, suffix[i]/suffix[i-1])   # 가중 분할(HUD 큼)
            anchor = w
        # 우측 Stage / 하단 Console 등 기본 패널 숨김 -> 뷰포트+POV+HUD만 깔끔하게
        for wn in ("Stage", "Console", "Property", "Content", "Layer", "Render Settings", "Semantics Schema Editor"):
            try:
                _pw = ui.Workspace.get_window(wn)
                if _pw: _pw.visible = False
            except Exception: pass
        # Script Editor를 우측에 도킹(발표용 dt_scenario("intrusion") 입력)
        try:
            from isaacsim.core.utils.extensions import enable_extension as _en
            _en("omni.kit.window.script_editor")
            for _ in range(6): world.step(render=True)
            _se = ui.Workspace.get_window("Script Editor")
            if _se and vpw:
                _se.dock_in(vpw, ui.DockPosition.RIGHT, 0.26); _se.visible = True
                print(">>> Script Editor 우측 도킹")
        except Exception as ex: print("script editor dock skip:", ex)
        for _ in range(3): world.step(render=True)
        print(">>> POV/HUD 좌측 균등 도킹 + 패널 숨김", items)
    except Exception as ex: print("dock skip:", ex)
    # 시작 뷰포인트 — 모든 창/도킹 셋업 '뒤'에 설정(앞서 설정 시 POV/도킹이 덮어쓰는 문제 방지)
    # 시작 뷰포인트: USD에 저장한 /World/Camera가 있으면 메인 뷰포트가 그걸 그대로 사용(정확히 그 뷰, roll 포함).
    # 없으면 환경변수 eye/target으로 set_camera_view 폴백.
    DEMO_CAM = os.environ.get("DEMO_CAM_PRIM", "/World/Camera")
    _cam_done = False
    try:
        from omni.kit.viewport.utility import get_viewport_from_window_name, get_active_viewport
        _mvp = get_viewport_from_window_name("Viewport") or get_active_viewport()
        if stage.GetPrimAtPath(DEMO_CAM).IsValid() and _mvp is not None:
            _mvp.set_active_camera(DEMO_CAM)
            print(">>> 시작 뷰포인트 = %s (USD 저장 카메라)" % DEMO_CAM); _cam_done = True
    except Exception as ex: print("active cam skip:", ex)
    if not _cam_done:
        CAM_EYE = list(map(float, os.environ.get("DEMO_CAM_EYE", "-3000,4500,2525").split(",")))
        CAM_TGT = list(map(float, os.environ.get("DEMO_CAM_TGT", "1375,300,2525").split(",")))
        try:
            from isaacsim.core.utils.viewports import set_camera_view
            set_camera_view(eye=CAM_EYE, target=CAM_TGT, camera_prim_path="/OmniverseKit_Persp")
            print(">>> 시작 뷰포인트(폴백)", CAM_EYE, "->", CAM_TGT)
        except Exception as ex: print("set_camera_view skip:", ex)
    for _ in range(3): world.step(render=True)
    # 수상자 머리 위 world-anchored 3D 라벨 — 메인 뷰 + Husky POV 둘 다
    try:
        from omni.kit.viewport.utility import get_active_viewport_window
        from omni.ui import scene as _sc
        import omni.ui as _oui
        def _mk_label(winname, key):
            w = get_active_viewport_window(winname)
            if w is None: return
            with w.get_frame("dt_lbl_" + key):
                sv = _sc.SceneView()
                with sv.scene:
                    LABEL3D[key + "_xf"] = _sc.Transform()
                    with LABEL3D[key + "_xf"]:
                        LABEL3D[key + "_txt"] = _sc.Label("", alignment=_oui.Alignment.CENTER_BOTTOM,
                                                          color=_oui.color(1.0, 0.15, 0.15), size=24)
            try: w.viewport_api.add_scene_view(sv)
            except Exception: pass
            LABEL3D[key + "_sv"] = sv
        _mk_label("Viewport", "main")
        _mk_label("Husky POV", "pov")
        # 로봇 머리 위 라벨(ROBOT 1/2/3) — 메인 뷰포트에 어느 Husky가 누군지 표시
        try:
            wv = get_active_viewport_window("Viewport")
            if wv is not None:
                with wv.get_frame("dt_robot_lbls"):
                    rsv = _sc.SceneView()
                    with rsv.scene:
                        for ri, rb in enumerate(robots):
                            xf = _sc.Transform(); LABEL3D["rb_xf_%d" % ri] = xf
                            with xf:
                                LABEL3D["rb_txt_%d" % ri] = _sc.Label(
                                    "ROBOT %d%s" % (ri + 1, " CAM" if (rb.has_cam or DEMO_MULTICAM) else ""),
                                    alignment=_oui.Alignment.CENTER_BOTTOM,
                                    color=_oui.color(0.3, 0.9, 1.0), size=22)
                wv.viewport_api.add_scene_view(rsv); LABEL3D["rb_sv"] = rsv
        except Exception as ex: print("robot label skip:", ex)
        print(">>> 3D 라벨 준비 (수상자 main+POV + 로봇 ROBOT 1/2/3)")
    except Exception as ex: print("3D label skip:", ex)

def _rect(S, E, C, b, y, col):
    pts = [(b["xmin"],y,b["zmin"]),(b["xmax"],y,b["zmin"]),(b["xmax"],y,b["zmax"]),(b["xmin"],y,b["zmax"])]
    for i in range(4):
        S.append(pts[i]); E.append(pts[(i+1)%4]); C.append(col)

def _fov_clip(x, z, ang, maxd, step=25.0):
    """FOV 레이를 navmesh 밖(벽)에서 멈춤. snap 변위>25 = off-mesh(벽). 촘촘히 스텝."""
    d = step
    while d <= maxd:
        px = x + math.sin(ang)*d; pz = z + math.cos(ang)*d
        cp = snap(px, pz)
        cx = getattr(cp, "x", px); cz = getattr(cp, "z", pz)
        if math.hypot(cx-px, cz-pz) > 25.0:
            return max(0.0, d-step)
        d += step
    return maxd

def draw_boxes():
    if DRAW is None: return
    DRAW.clear_lines()
    starts = []; ends = []; cols = []; GY = 6.0
    # 구역 바닥 오버레이: core(빨강)/perimeter(주황)/office(파랑)
    for zz in ZONES.get("zones", []):
        b = zz.get("bounds")
        if not b: continue
        t = zz.get("type", "")
        col = (1.0,0.15,0.15,1.0) if t=="restricted_area" else \
              (1.0,0.6,0.0,1.0) if t=="sensitive_perimeter" else \
              (0.3,0.5,1.0,0.7) if t=="office" else (0.4,0.4,0.4,0.5)
        _rect(starts, ends, cols, b, GY, col)
    # 출구 마커(세로 기둥): 차단되면 초록, 아니면 노랑
    blocked = {rb.intercept_exit for rb in robots if rb.mode == "BLOCK"}
    for e in EXITS_CFG:
        ex, ez = float(e["x"]), float(e["z"]); eid = e.get("id","")
        col = (0.1,1.0,0.2,1.0) if eid in blocked else (1.0,0.9,0.1,1.0)
        starts.append((ex,GY,ez)); ends.append((ex,GY+260,ez)); cols.append(col)
    # FOV 콘 — 외곽선만(양 가장자리+끝 호), 짧은 시각범위(혼잡↓). 벽에서 클립. DEMO_CONE=0 끄기.
    import math as _m
    if DEMO_CONE:
        for rb in robots:
            if not (rb.has_cam or DEMO_MULTICAM): continue   # 1대=robot0만, 3대=카메라 있는 셋 다
            ccol = (0.2, 0.8, 1.0, 0.55)
            apex = (rb.x, FIXED_Y + 20, rb.z)
            pts = []
            for s in [i/8.0 for i in range(-8, 9)]:          # -1..1, 17점(호)
                a = rb.phi + s*_m.radians(SENSE_FOV); dd = _fov_clip(rb.x, rb.z, a, SENSE_RANGE)   # 콘=실제 탐지범위
                pts.append((rb.x+_m.sin(a)*dd, FIXED_Y+20, rb.z+_m.cos(a)*dd))
            starts.append(apex); ends.append(pts[0]);  cols.append(ccol)   # 왼쪽 가장자리
            starts.append(apex); ends.append(pts[-1]); cols.append(ccol)   # 오른쪽 가장자리
            for i in range(len(pts)-1):                       # 끝 호(벽클립 반영해 꺾임)
                starts.append(pts[i]); ends.append(pts[i+1]); cols.append(ccol)
    # 차단 경로(로봇->목표 출구): 자홍
    for rb in robots:
        if rb.mode in ("INTERCEPT","BLOCK") and rb.goal is not None:
            starts.append((rb.x,FIXED_Y+15,rb.z)); ends.append((rb.goal[0],GY+40,rb.goal[1]))
            cols.append((1.0,0.2,1.0,0.9))
    # 사람 박스 + 도주경로
    for p in people:
        px, pz = p.pos(); y0 = PERSON_Y; y1 = PERSON_Y + 185.0; hw = 48.0
        # 등급 진행 시각화: 정상(초록) -> 평가중/watch(노랑) -> 수상확정(빨강)
        if p.suspicious:                         col = (1.0, 0.1, 0.1, 1.0)   # 수상(빨강)
        elif getattr(p, "assessing", False):     col = (1.0, 0.8, 0.0, 1.0)   # 평가중(노랑)
        else:                                    col = (0.1, 1.0, 0.2, 1.0)   # 정상(초록)
        c = [(px-hw,y0,pz-hw),(px+hw,y0,pz-hw),(px+hw,y0,pz+hw),(px-hw,y0,pz+hw),
             (px-hw,y1,pz-hw),(px+hw,y1,pz-hw),(px+hw,y1,pz+hw),(px-hw,y1,pz+hw)]
        for a,b in [(0,1),(1,2),(2,3),(3,0),(4,5),(5,6),(6,7),(7,4),(0,4),(1,5),(2,6),(3,7)]:
            starts.append(c[a]); ends.append(c[b]); cols.append(col)
        if p.suspicious:
            starts.append((robot.x, FIXED_Y+30, robot.z)); ends.append((px, y0+90, pz)); cols.append((1.0,0.6,0.0,1.0))
        if p.behavior == "flee" and getattr(p, "flee_wp", None):
            wp = p.flee_wp
            for i in range(len(wp)-1):
                starts.append((wp[i][0],GY+10,wp[i][1])); ends.append((wp[i+1][0],GY+10,wp[i+1][1]))
                cols.append((1.0,0.3,0.0,0.9))
    if starts:
        DRAW.draw_lines(starts, ends, cols, [3.0]*len(starts))

def update_hud(elapsed):
    if HUD is None: return
    s, e = HUD
    nsus = sum(1 for p in people if p.suspicious)
    nblock = sum(1 for rb in robots if rb.mode == "BLOCK")
    scn_name = SCENARIOS[scn_i[0]]["name"] if SCENARIOS else ""
    # AI 판단 라인: 수상 확정 > 평가중 > 전원 정상
    sp = next((p for p in people if p.suspicious), None)
    ap = next((p for p in people if getattr(p, "assessing", False)), None)
    if sp is not None:
        _vs = getattr(sp, "vlm_score", 0.0)
        if getattr(sp, "vlm_state", "") == "done" and _vs >= VLM_SUSPECT:
            ai = "AI: SUSPICIOUS %.2f - %s" % (_vs, getattr(sp, "vlm_reason", "") or "judging")
        else:   # 체류(zone+dwell)로 확정 — VLM 미완/저점수일 때 모순 표시 방지
            _z = zone_of(sp.x, sp.z)
            ai = "AI: SUSPICIOUS - loitering %.0fs @ %s" % (getattr(sp, "seen", 0.0), _z.get("id", "restricted area"))
    elif ap is not None:
        ai = "AI: assessing %s ..." % ap.id
    else:
        ai = "AI: all clear"
    s.text = "SCENARIO: %s    t=%.1fs\nSUSPECT %d/%d  BLOCKING %d/%d  %s\n%s" % (
        scn_name, elapsed, nsus, len(people), nblock, len(robots), intercept_status[0], ai)
    lines = []
    for ev in events[-8:][::-1]:
        lines.append("[%6.1fs] %s\n    %s" % (ev.get("t",0), ev.get("text",""), ev.get("judgment","")))
    e.text = "\n".join(lines) if lines else "(no events)"

# ---------- 로그 ----------
event_log = []
_logf = open(LOG_PATH, "w")
_logf.write("# GIST X+AI DT 순찰 로그\n"); _logf.flush()
def log_event(msg):
    event_log.append(msg); print(msg)
    _logf.write(msg + "\n"); _logf.flush()

# 구조화 이벤트(대시보드: 클릭->캡처+위치) + 증거 캡처
events = []
EVID_DIR = P("evidence")
os.makedirs(EVID_DIR, exist_ok=True)
latest_frame = [None]   # 최신 카메라 프레임(박스 포함)
_evn = [0]

# ---------- SQLite 이벤트 DB (사진경로·위치·vlm·차단결과 영구 저장) ----------
import sqlite3 as _sql, time as _time
DB_PATH = P("events.db")
try:
    _dbc = _sql.connect(DB_PATH)
    _dbc.execute("""CREATE TABLE IF NOT EXISTS events(
        id INTEGER PRIMARY KEY AUTOINCREMENT, t REAL, wall TEXT, scenario TEXT,
        person TEXT, behavior TEXT, zone TEXT, sx REAL, sz REAL,
        robot TEXT, rx REAL, rz REAL, vlm_score REAL, judgment TEXT, img TEXT)""")
    _dbc.execute("""CREATE TABLE IF NOT EXISTS interceptions(
        id INTEGER PRIMARY KEY AUTOINCREMENT, t REAL, wall TEXT, suspect TEXT,
        predicted_exit TEXT, contained INTEGER, breaches TEXT, assignments TEXT)""")
    _dbc.commit(); print(">>> SQLite connected:", DB_PATH)
except Exception as _ex:
    print("sqlite skip:", _ex); _dbc = None
def db_event(ev, rid, rx, rz, scn):
    if _dbc is None: return
    try:
        _dbc.execute("INSERT INTO events(t,wall,scenario,person,behavior,zone,sx,sz,robot,rx,rz,vlm_score,judgment,img)"
                     " VALUES(?,?,?,?,?,?,?,?,?,?,?,?,?,?)",
            (ev["t"], _time.strftime("%Y-%m-%d %H:%M:%S"), scn, ev["person"], ev["behavior"], ev["zone"],
             ev["x"], ev["z"], rid, rx, rz, ev["vlm_score"], ev["judgment"], ev["img"]))
        _dbc.commit()
    except Exception: pass
def db_intercept(plan, suspect_id, t):
    if _dbc is None: return
    try:
        _dbc.execute("INSERT INTO interceptions(t,wall,suspect,predicted_exit,contained,breaches,assignments)"
                     " VALUES(?,?,?,?,?,?,?)",
            (t, _time.strftime("%Y-%m-%d %H:%M:%S"), suspect_id, plan.predicted_exit,
             1 if plan.contained else 0, str(plan.breaches),
             str([(a.robot_id, a.exit_id, a.role) for a in plan.assignments])))
        _dbc.commit()
    except Exception: pass
def record_event(p, elapsed):
    _evn[0] += 1
    img_rel = ""
    if latest_frame[0] is not None:
        try:
            import cv2 as _cv2
            fn = "ev_%03d.png" % _evn[0]
            _cv2.imwrite(EVID_DIR + "/" + fn, latest_frame[0]); img_rel = "evidence/" + fn
        except Exception: pass
    zid = zone_of(p.x, p.z).get("id", "")
    txt = "SUSPICIOUS %s — %s (%.1fs)" % (p.id, p.behavior, p.seen)   # 구역은 VLM 판정에 표시(중복 제거)
    ev = {"t": elapsed, "person": p.id, "x": p.x, "z": p.z,
          "behavior": p.behavior, "zone": zid, "text": txt, "img": img_rel,
          "judgment": getattr(p, "vlm_reason", "") or "AI analyzing...",
          "vlm_score": getattr(p, "vlm_score", 0.0)}
    events.append(ev)
    log_event("[%6.1fs] %s @ (%.0f,%.0f) vlm=%.2f" % (elapsed, txt, p.x, p.z, getattr(p, "vlm_score", 0.0)))
    # SQLite 적재: 탐지 로봇 + 그 위치 + 시나리오
    _rid = observed_by.get(p.id, "")
    _rb = next((r for r in robots if r.id == _rid), None)
    _scn = SCENARIOS[scn_i[0]]["name"] if "SCENARIOS" in globals() and SCENARIOS else ""
    db_event(ev, _rid, _rb.x if _rb else 0.0, _rb.z if _rb else 0.0, _scn)

# ---------- VLM(GPT-4o): 수상도 SCORE + 한 줄(영문) — 판단에 '직접' 입력 ----------
import base64 as _b64, urllib.request as _ureq, threading as _th, re as _re
def _read_key():
    try:
        k = open(P(".openai_key")).read().strip()
        if k: return k
    except Exception: pass
    return os.environ.get("OPENAI_API_KEY", "")
OPENAI_KEY = _read_key(); USE_VLM = bool(OPENAI_KEY)
VLM_MODEL = os.environ.get("DEMO_VLM_MODEL", "gpt-4o-mini")   # 실험: DEMO_VLM_MODEL=gpt-4o
VLM_PROMPT = os.environ.get("DEMO_VLM_PROMPT", "new")          # 실험: old=맥락없는 기본, new=체류·구역·rubric 주입
VLM_SUSPECT = float(os.environ.get("DEMO_VLM_SUSPECT", "0.5"))   # 이 점수 이상이면 VLM이 '수상' 판정
print(">>> VLM:", ("OpenAI connected (%s)" % VLM_MODEL) if USE_VLM else "no key -> mock score")
def _mock_vlm(p):
    z = zone_of(p.x, p.z)
    base = {"idle":0.45,"pace":0.7,"wander":0.6,"loop":0.4,"flee":0.95}.get(p.behavior, 0.4)
    if z.get("type") == "sensitive_perimeter": base = min(1.0, base + 0.25)
    if z.get("access_level") == "restricted": base = 1.0
    rsn = {"idle":"loitering in place","pace":"pacing back and forth","wander":"wandering",
           "loop":"repeated looping","flee":"fleeing"}.get(p.behavior, "unusual movement")
    return base, "%s @ %s" % (rsn, z.get("id", "area"))
def _vlm_call(img_path, p):
    with open(img_path, "rb") as f:
        b64 = _b64.b64encode(f.read()).decode()
    z = zone_of(p.x, p.z)
    dwell = getattr(p, "seen", 0.0)
    _restr = z.get("access_level") == "restricted" or z.get("type") == "sensitive_perimeter"
    _ctx = "a restricted/sensitive zone" if _restr else "a public/common area"   # 실제 zone 반영(정상인 왜곡 방지)
    # 맥락 주입: VLM은 1프레임만 보므로 체류시간·구역·점수기준을 근거로 제공(실제 사실 -> 일관된 판정)
    prompt = ("Datacenter security camera. The person is in '%s' (access=%s) — %s — "
              "and has remained here about %.0f seconds (movement=%s). "
              "How suspicious is this for casing/loitering a restricted area? "
              "Scale: 0.0-0.3 normal or just passing, 0.4-0.6 lingering, 0.7-1.0 loitering/casing. "
              "Reply ONE line exactly: 'SCORE=<0..1> | <reason <=6 words>'."
              % (z.get("id",""), z.get("access_level",""), _ctx, dwell, p.behavior))
    body = {"model": VLM_MODEL, "max_tokens": 40, "messages": [{"role":"user","content":[
        {"type":"text","text":prompt},
        {"type":"image_url","image_url":{"url":"data:image/png;base64,"+b64}}]}]}
    req = _ureq.Request("https://api.openai.com/v1/chat/completions",
        data=json.dumps(body).encode(),
        headers={"Authorization":"Bearer "+OPENAI_KEY,"Content-Type":"application/json"})
    with _ureq.urlopen(req, timeout=25) as r:
        out = json.loads(r.read().decode())["choices"][0]["message"]["content"].strip()
    m = _re.search(r"([01](?:\.\d+)?)", out)
    score = max(0.0, min(1.0, float(m.group(1)))) if m else 0.5
    reason = (out.split("|",1)[1].strip() if "|" in out else out)[:38]
    return score, reason
def request_vlm(p, img_path):
    """관측 중인 사람의 장면을 GPT-4o로 평가 -> vlm_score/reason (비동기, 판단에 입력)."""
    p.vlm_state = "pending"
    def work():
        try:
            s, rsn = _vlm_call(img_path, p) if (USE_VLM and img_path) else _mock_vlm(p)
        except Exception as ex:
            s, rsn = _mock_vlm(p); print("VLM err:", ex)
        p.vlm_score = s; p.vlm_reason = "AI: %s (%.2f)" % (rsn, s); p.vlm_state = "done"
    _th.Thread(target=work, daemon=True).start()
def process_vlm(elapsed):
    for e in events:
        if str(e.get("judgment","")).startswith("AI analyzing"):
            p = next((pp for pp in people if pp.id == e["person"]), None)
            if p is not None and getattr(p, "vlm_state", "none") == "done":
                e["judgment"] = p.vlm_reason; e["vlm_score"] = p.vlm_score

# 물리 레이캐스트 가림: robot→사람 직선에 벽 충돌이 있으면 안 보임(navmesh와 별개, 정확).
_PHYSXQ = None
def _physxq():
    global _PHYSXQ
    if _PHYSXQ is None:
        try:
            from omni.physx import get_physx_scene_query_interface
            _PHYSXQ = get_physx_scene_query_interface()
        except Exception: _PHYSXQ = False
    return _PHYSXQ or None
OCCLUSION = os.environ.get("DEMO_OCCLUSION", "1") == "1"   # 벽 가림(레이캐스트). 발각이 늦으면 DEMO_OCCLUSION=0
def _occluded(rb, p, dist):
    """robot→사람 직선에 벽 충돌이 있으면 True. 충돌지오 없으면(raycast 무응답) False(폴백=무회귀)."""
    if not OCCLUSION: return False
    q = _physxq()
    if q is None or dist < 120: return False
    try:
        import carb
        dx, dz = (p.x-rb.x)/dist, (p.z-rb.z)/dist
        ox, oz = rb.x + dx*100.0, rb.z + dz*100.0          # 로봇 몸체 자기충돌 회피(100cm 앞 시작)
        rng = dist - 100.0 - 70.0                           # 사람 70cm 앞에서 종료(사람 자체 충돌 회피)
        if rng <= 0: return False
        hit = q.raycast_closest(carb.Float3(ox, FIXED_Y+90.0, oz), carb.Float3(dx, 0.0, dz), rng)
        return bool(hit and hit.get("hit"))
    except Exception: return False

def robot_sees(rb, p):
    dx, dz = p.x - rb.x, p.z - rb.z
    d = math.hypot(dx, dz)
    if d > SENSE_RANGE: return False
    ang = abs(math.degrees(wrap(math.atan2(dx, dz) - rb.phi)))
    if ang >= SENSE_FOV: return False
    return not _occluded(rb, p, d)

# 카메라 탐지용 perception wrapper (GUI/headless 공통)
import cv2
from perception import PersonPerception
from suspicion import SuspicionEngine
person_perception = PersonPerception(min_det=0.4, min_track=0.4, complexity=1, det_threshold=0.35)
# 사람별 판단 엔진(체류·구역·행동 + VLM 신호 융합). VLM을 판단에 직접 입력(가중치 0.30).
_W = {"dwell":0.35, "zone":0.18, "behavior":0.10, "time":0.04, "face":0.03, "vlm":0.30}
engines = {p.id: SuspicionEngine(suspect_secs=SUSPECT_SECS, watch_secs=0.5, use_vlm=True, weights=_W)
           for p in people}

# ---- attribution 상태(#1): 사람별로 어느 카메라가 봤는가 + 신선도(#2) ----
observed_id = [None]       # 가장 최근 관측 사람(HUD/진단용)
cam_seen_t = {}            # pid -> 마지막 관측 시각
observed_by = {}           # pid -> 관측한 로봇 id
last_det = [None]
_capn = [0]; _detn = [0]; _dbg_t = [0.0]   # 진단: 캡처/탐지 카운트
_DBG = open("/tmp/dt_debug.txt", "w")
def dbg(elapsed):
    if elapsed - _dbg_t[0] < 2.0: return
    _dbg_t[0] = elapsed
    ppl = " ".join("%s[%s s%.1f %s]" % (p.id, getattr(p, "level", "?"), p.seen, p.behavior) for p in people)
    rbs = " ".join("%s:%s" % (rb.id.split("_")[-1], rb.mode) for rb in robots)
    _DBG.write("[%6.1fs] cap=%d det=%d obs=%s | R[%s] | %s\n" % (
        elapsed, _capn[0], _detn[0], observed_id[0], rbs, ppl)); _DBG.flush()
def _expected_norm_x(rb, p):
    dx, dz = p.x - rb.x, p.z - rb.z
    bearing = wrap(math.atan2(dx, dz) - rb.phi)            # rad, 0=정면
    return 0.5 + 0.5 * (bearing / max(math.radians(SENSE_FOV), 1e-3))   # 화면 가로 0..1 근사
def attribute_observed(det, det_rb, elapsed):
    """det(perception)을 그 로봇 FOV 후보 중 bbox 가로위치로 매칭 -> 그 사람 관측 기록."""
    if det is None or det_rb is None:
        return
    last_det[0] = det
    cands = [p for p in people if robot_sees(det_rb, p)]
    if det.get("present") and cands:
        cn = det.get("signals", {}).get("bbox_center_norm", (0.5, 0.5))
        nx = cn[0] if isinstance(cn, (list, tuple)) else 0.5
        pp = min(cands, key=lambda p: abs(_expected_norm_x(det_rb, p) - nx))
        pid = pp.id
        cam_seen_t[pid] = elapsed
        observed_by[pid] = det_rb.id
        observed_id[0] = pid
        # 순찰 중 민감구역(datacenter 경계/내부)의 사람을 발견하면 멈춰서 관찰 -> 체류 누적/판정
        # (door-to-door 순찰처럼 지나가도 발각되게 함. 정상인은 public이라 트리거 안 됨)
        # 정보 공유(ownership): 다른 로봇이 이미 이 사람을 관측(assessing)/추격 중이면 중복 관측 안 함.
        #   - cam_seen_t/엔진/vlm_state는 사람단위 공유 -> 여러 카메라가 봐도 관측치는 합쳐짐(공유)
        #   - 단 '멈춰서 검문(observe)'은 한 대만 -> 로봇들이 한 사람에 몰리는 중복 제거
        #   - 민감구역 위협은 여러 로봇이 함께 관측(협동 판단). 일반인(public)은 한 대만(중복 방지).
        _sens_pp = zone_of(pp.x, pp.z).get("type") in ("sensitive_perimeter", "restricted_area")
        owned_by_other = (not _sens_pp) and any(
            rb is not det_rb and rb.observe_id == pp.id and rb.mode in ("OBSERVE", "PURSUE") for rb in robots)
        already_judged = (pp.vlm_state == "done") or pp.suspicious   # clear(판정완료) 또는 이미 수상이면 스킵
        # 순찰 중 '아직 판정 안 된 + 아무도 안 맡은' 사람만 멈춰서 관찰(=검문). 도주 중 수상자는 제외(추격 유지).
        if det_rb.mode == "PATROL" and not already_judged and not owned_by_other \
                and not (flee_active[0] and pp.behavior == "flee"):
            det_rb.observe(pp)

import interception as icp
FLEE_SPEED = 260.0          # 도주(빠름; 논리위치를 이 속도로 이동 + biped 텔레포트). 순찰200<도주260<차단380
flee_active = [False]       # 데모: 수상자 1명 도주/차단 1회
intercept_status = [""]     # HUD용 차단 상태 문구
def nav_dist(a, b):
    """navmesh 최단경로 길이(cm). interception에 주입 — 벽 우회 실제거리."""
    try:
        path = navmesh.query_shortest_path(snap(a[0], a[1]), snap(b[0], b[1]), agent_radius=5.0)
        if path:
            pts = path.get_points()
            if pts and len(pts) >= 2:
                return sum(math.hypot(pts[i+1][0]-pts[i][0], pts[i+1][2]-pts[i][2]) for i in range(len(pts)-1))
    except Exception: pass
    return math.hypot(a[0]-b[0], a[1]-b[1])
def start_flee(p):
    if p.behavior == "flee": return
    exits = [(float(e["x"]), float(e["z"])) for e in EXITS_CFG] or [(p.x, p.z)]
    tgt = min(exits, key=lambda e: nav_dist((p.x, p.z), e))
    wp = navpath(snap(p.x, p.z), snap(tgt[0], tgt[1]), r=5.0)
    p.behavior = "flee"; p.flee_target = tgt; p.flee_wp = wp; p.flee_i = 1; p._rerouted = False
    log_event(">>> FLEE %s -> 최근접출구 (%.0f,%.0f)" % (p.id, tgt[0], tgt[1]))
def _exit_blocked(ex):
    return any(rb.mode == "BLOCK" and math.hypot(rb.x-ex[0], rb.z-ex[1]) < 260.0 for rb in robots)
def _exit_robot(ex):
    """그 출구를 막고 있는(BLOCK) 로봇 — 도주자가 그 앞에서 멈추도록."""
    c = [rb for rb in robots if rb.mode == "BLOCK" and math.hypot(rb.x-ex[0], rb.z-ex[1]) < 260.0]
    return min(c, key=lambda rb: math.hypot(rb.x-ex[0], rb.z-ex[1])) if c else None
def nearest_unblocked_exit(x, z):
    cand = [(float(e["x"]), float(e["z"])) for e in EXITS_CFG if not _exit_blocked((float(e["x"]), float(e["z"])))]
    return min(cand, key=lambda e: nav_dist((x, z), e)) if cand else None
def dispatch_interception(p, pursuer):
    # 발각한 로봇은 추격 전담 -> 차단 후보에서 제외. 나머지가 출구 배정.
    irobots = [icp.Robot(id=rb.id, x=rb.x, z=rb.z, speed=INTERCEPT_SPEED,
                         available=(rb is not pursuer)) for rb in robots]
    sus = icp.Suspect(x=p.x, z=p.z, speed=FLEE_SPEED, confidence=1.0)
    plan = icp.plan_interception(irobots, icp.exits_from_config(ZONES), sus, dist=nav_dist)
    rbmap = {rb.id: rb for rb in robots}
    for a in plan.assignments:
        rb = rbmap.get(a.robot_id)
        if rb is not None and a.role == "block":
            rb.go_intercept(a.exit_id, a.target)
    intercept_status[0] = "INTERCEPT %s | flee->%s | %s" % (
        "CONTAINED" if plan.contained else "BREACH",
        plan.predicted_exit,
        ", ".join("%s->%s" % (a.robot_id.split("_")[-1], a.exit_id) for a in plan.assignments if a.role == "block"))
    log_event(">>> INTERCEPT pred=%s contained=%s breaches=%s | %s" % (
        plan.predicted_exit, plan.contained, plan.breaches,
        [(a.robot_id, a.exit_id, a.role, round(a.robot_eta,1)) for a in plan.assignments]))
    return plan

# ---------- 시나리오 2개 번갈아 재생 (정상 ↔ 침입) ----------
SCENARIOS = [
    {"name": "NORMAL",    # 수상자 없음: 전원 공개/사무 구역을 걸어다님 -> 아무도 발각 안 됨
     "people": [(1500, 2600, "walk", WALK_ROUTES["central"]),
                (1100, 1000, "walk", WALK_ROUTES["south"]),
                (700, 2300, "walk", WALK_ROUTES["west"])],
     "duration": 30.0, "until_contained": False},
    {"name": "INTRUSION", # 수상자만 datacenter perimeter에서 살핌(casing). 정상인은 걸어다님 -> 발각/도주/차단
     "people": [(1450, 2900, "infiltrate", SUSPECT_ROUTE),           # P1 수상자 — datacenter 근처서 짧게 잠입 -> casing -> 발각
                (1100, 1000, "walk", WALK_ROUTES["south"]),          # P2 정상 보행
                (700, 2300, "walk", WALK_ROUTES["west"])],           # P3 정상 보행
     "duration": 110.0, "until_contained": True},
]
scn_i = [0]; scn_t0 = [0.0]; _caught_t = [None]   # 검거 시점(전환 grace용)
DEMO_ONLY = os.environ.get("DEMO_ONLY", "").lower()   # ""=순환, "normal"/"intrusion"=한 시나리오만 반복(분리 녹화용)
_only_idx = 1 if DEMO_ONLY == "intrusion" else 0
def apply_scenario(idx, elapsed):
    scn = SCENARIOS[idx % len(SCENARIOS)]
    for i, p in enumerate(people):
        if i < len(scn["people"]):
            c = scn["people"][i]
            p.reconfigure(c[0], c[1], c[2], c[3] if len(c) > 3 else None)
        engines[p.id] = SuspicionEngine(suspect_secs=SUSPECT_SECS, watch_secs=0.5, use_vlm=True, weights=_W)
    for rb in robots:                       # 시나리오 재시작 시 모든 로봇 시작위치로 초기화
        rb.mode = "PATROL"; rb.goal = None; rb.goal_path = []
        rb.pursue_target = None; rb.intercept_exit = None
        if getattr(rb, "route_door", None): rb.route = rb.route_door   # 구역 로봇은 문 경로로 복귀
        rb.x, rb.z = rb.start_xz; rb._apply(); rb.pi = 1               # 위치 초기화(카메라 포함)
    flee_active[0] = False; intercept_status[0] = ""; _caught_t[0] = None
    observed_id[0] = None; cam_seen_t.clear(); observed_by.clear()
    scn_t0[0] = elapsed
    log_event(">>> ===== SCENARIO: %s =====" % scn["name"])

print(">>> 데모 시작 (headless=%s)" % HEADLESS)
last_cap = -9.0; t_prev = time.perf_counter(); t_start = t_prev; frame_idx = 0; _cam_dump = [0.0]; _fps_acc = [0.0, 0]
scn_i[0] = _only_idx if DEMO_ONLY else 0
apply_scenario(scn_i[0], 0.0)   # 첫 시나리오 (DEMO_ONLY면 해당 시나리오만)
# ── 발표용 수동 전환: Isaac Script Editor에서 dt_scenario("intrusion"/"normal") 호출, 또는 파일 /tmp/dt_cmd ──
_manual_cmd = [None]
def dt_set_scenario(name):
    _manual_cmd[0] = str(name).strip().lower(); return "queued: %s" % _manual_cmd[0]
try:
    import builtins as _bi
    _bi.dt_scenario = dt_set_scenario   # builtins=전역 -> Script Editor 네임스페이스에서도 호출 가능
    print(">>> 수동 전환 준비: Script Editor에 dt_scenario(\"intrusion\") 또는 echo intrusion>/tmp/dt_cmd")
except Exception as ex: print("manual scenario skip:", ex)
while simulation_app.is_running():
    now = time.perf_counter(); dt = now - t_prev; t_prev = now
    _fps_acc[0] += dt; _fps_acc[1] += 1                       # raw 프레임시간 누적 -> 실측 fps
    if _fps_acc[0] >= 2.0:
        print(">>> fps=%.1f (멀티캠=%s)" % (_fps_acc[1]/_fps_acc[0], DEMO_MULTICAM)); _fps_acc[0] = 0.0; _fps_acc[1] = 0
    if dt > 0.1: dt = 0.1
    elapsed = now - t_start

    # 수동 전환(발표용): dt_scenario("normal"/"intrusion") 또는 파일 /tmp/dt_cmd -> 즉시 해당 시나리오
    _mc = _manual_cmd[0]; _manual_cmd[0] = None
    if _mc is None:
        try:
            if os.path.exists("/tmp/dt_cmd"):
                _mc = open("/tmp/dt_cmd").read().strip().lower(); os.remove("/tmp/dt_cmd")
        except Exception: _mc = None
    if _mc in ("normal", "intrusion"):
        scn_i[0] = 0 if _mc == "normal" else 1
        apply_scenario(scn_i[0], elapsed)

    # 시나리오 전환: 침입은 '실제 검거(cornered)' 후 잠깐 보여주고 전환(reroute 허용). 정상은 시간 경과 후.
    _scn = SCENARIOS[scn_i[0]]; _sdt = elapsed - scn_t0[0]
    _switch = _sdt > _scn["duration"]                  # 시간 초과(안전망)
    if _scn["until_contained"]:
        if any(getattr(p, "caught", False) for p in people):
            if _caught_t[0] is None: _caught_t[0] = elapsed       # 검거 시점 기록
            if elapsed - _caught_t[0] > 6.0: _switch = True       # 검거 후 6초 보여주고 전환
        # caught 전엔 시간초과로만 전환 -> reroute 중에는 안 끊김
    if _switch:
        if not DEMO_ONLY:
            scn_i[0] = (scn_i[0] + 1) % len(SCENARIOS)   # 순환
        apply_scenario(scn_i[0], elapsed)                # DEMO_ONLY면 같은 시나리오 반복(분리 녹화)

    # 경계 침입 자동 대응(확실한 발각): 민감구역의 '미판정' 사람을 가장 가까운 카메라 로봇이 출동해 검문.
    #   FOV로 우연히 잡길 기다리지 않고 -> 로봇이 가서 마주보고 관측(OBSERVE가 navmesh로 접근+응시+누적).
    for p in people:
        if zone_of(p.x, p.z).get("type") in ("sensitive_perimeter", "restricted_area") \
                and not p.suspicious and p.vlm_state != "done" \
                and not (flee_active[0] and p.behavior == "flee"):
            if not any(getattr(rb, "observe_id", None) == p.id and rb.mode in ("OBSERVE", "PURSUE") for rb in robots):
                _cams = [rb for rb in robots if rb.has_cam and rb.mode == "PATROL"]   # 카메라(R0)만 출동 -> R1/R2는 출구 지킴(reroute 보장)
                if _cams:
                    min(_cams, key=lambda rb: math.hypot(rb.x - p.x, rb.z - p.z)).observe(p)

    # 로봇 순찰 + (감지 시) 제자리 관측. 사람은 각자 행동.
    for rb in robots:
        obs = next((p for p in people if p.id == rb.observe_id), None)
        rb.step(dt, obs)
    for p in people:
        p.step(dt)

    # ---- 렌더 + 카메라 캡처(perception): 모든 카메라 처리(FOV에 사람 있는 것만 mediapipe) ----
    want = (elapsed - last_cap) >= (0.25 if DEMO_MULTICAM else 0.1)   # 3대 모드는 캡처 주기↓(부하↓)
    world.step(render=(want or not HEADLESS))   # GUI는 매 프레임, headless는 캡처 시에만
    if not HEADLESS and (elapsed - _cam_dump[0]) > 1.0:   # 현재 persp 카메라 eye/target을 파일로 -> 원하는 뷰 좌표 추출용
        _cam_dump[0] = elapsed
        try:
            _m = UsdGeom.Xformable(stage.GetPrimAtPath("/OmniverseKit_Persp")).ComputeLocalToWorldTransform(0)
            _e = _m.ExtractTranslation(); _r = _m.ExtractRotationMatrix()
            _f = Gf.Vec3d(-_r[2][0], -_r[2][1], -_r[2][2]); _t = _e + _f * 2000.0
            open("/tmp/cam_now.txt", "w").write('DEMO_CAM_EYE="%.0f,%.0f,%.0f" DEMO_CAM_TGT="%.0f,%.0f,%.0f"\n'
                                                % (_e[0], _e[1], _e[2], _t[0], _t[1], _t[2]))
        except Exception: pass
    if want and robot_cams:
        last_cap = elapsed
        susp_now = any(p.suspicious for p in people)
        for rb, _annot in robot_cams:
            if not any(robot_sees(rb, p) for p in people):
                continue                      # 이 카메라 FOV에 사람 없음 -> mediapipe 스킵(성능)
            rgb = None
            try:
                data = _annot.get_data()
                if data is not None and getattr(data, "size", 0) > 0:
                    rgb = data[:, :, :3]
            except Exception:
                rgb = None
            if rgb is None or getattr(rgb, "size", 0) == 0:
                continue
            frame = cv2.cvtColor(rgb.astype(np.uint8), cv2.COLOR_RGB2BGR)
            det = person_perception.detect(frame)
            _capn[0] += 1
            if det.get("present"): _detn[0] += 1
            attribute_observed(det, rb, elapsed)
            # POV/증거 프레임: 로봇0 우선, 아니면 탐지한 카메라
            if rb is robots[0] or latest_frame[0] is None or det.get("present"):
                if det.get("bbox") is not None:
                    x1, y1, x2, y2 = det["bbox"]
                    vis = float(det.get("vis", 0.0)); quality = float(det.get("pose_quality", vis))
                    person_perception.draw_skeleton(frame, det)
                    col = (0,0,255) if susp_now else (0,255,0)
                    cv2.rectangle(frame,(x1,y1),(x2,y2),col,2)
                    cv2.putText(frame,("SUSPICIOUS" if susp_now else "PERSON")+" q%.2f v%.2f"%(quality,vis),
                                (x1,max(12,y1-6)),cv2.FONT_HERSHEY_SIMPLEX,0.55,col,2)
                cv2.rectangle(frame,(0,0),(frame.shape[1],24),(0,0,0),-1)
                cv2.putText(frame,"%s:%s  t=%.1fs"%(rb.id.split("_")[-1], rb.mode, elapsed),(6,17),
                            cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,255,0),1)
                latest_frame[0] = frame.copy()
                cv2.imwrite(P("cam_live.jpg"), frame)
        frame_idx += 1

    # ---- 수상 판정: 사람별 SuspicionEngine (구역·체류·행동 신호 융합) ----
    for p in people:
        _gap = elapsed - cam_seen_t.get(p.id, -9.0)                 # 마지막 관측 이후 경과
        seen_now = _gap < 0.35                                      # 어느 카메라든 최근 관측(#2 신선도)
        # 재판단: clear된 사람을 REJUDGE_GAP초 이상 놓쳤다 다시 만나면 판정 리셋(재식별->재평가).
        # 지속 관측 중(gap 작음)에는 리셋 안 함. 수상 확정자는 유지(latch).
        if _gap > REJUDGE_GAP and p.vlm_state == "done" and not p.suspicious:
            p.vlm_state = "none"; p.vlm_score = 0.0; p.vlm_reason = ""; p._assess_t = 0.0
        # 관측 누적시간(엔진 seen_time은 3.0 캡 -> 별도 타이머). mediapipe(seen_now)가 들쭉날쭉해도
        # '카메라 로봇이 가까이서 관측(OBSERVE)' 중이면 관측으로 인정 -> 로봇이 보면 판단 진행(쳐다만 보는 버그 방지).
        _watching = any(getattr(rb, "observe_id", None) == p.id and rb.mode == "OBSERVE"
                        and math.hypot(rb.x - p.x, rb.z - p.z) < 450.0
                        for rb in robots if (rb.has_cam or DEMO_MULTICAM))
        if (seen_now or _watching) and not p.suspicious:
            p._assess_t = getattr(p, "_assess_t", 0.0) + dt
        elif not seen_now and not _watching:
            p._assess_t = max(0.0, getattr(p, "_assess_t", 0.0) - dt * 0.5)   # 아무도 안 보면 천천히 감소
        sig = zone_signals(p.x, p.z, p.behavior)
        sig["present"] = seen_now
        sig["vlm_score"] = p.vlm_score                            # VLM 점수를 판단에 직접 입력
        if seen_now and last_det[0] is not None:
            sig["face_occluded"] = bool(last_det[0].get("signals", {}).get("face_occluded", False))
        r = engines[p.id].update(sig, dt)
        p.seen = r["seen_time"]; p.level = r["level"]
        # VLM 요청/재요청: watch+ 도달 시. 민감구역인데 아직 <임계면 주기적 재요청(gpt-4o 점수 변동 보완).
        _sensitive = bool(sig.get("restricted_location") or sig.get("near_restricted_area"))
        _vlm_due = (p.vlm_state == "none")
        if (p.vlm_state == "done" and p.vlm_score < VLM_SUSPECT and _sensitive
                and (elapsed - getattr(p, "_vlm_t", -9.0)) > 2.0):
            _vlm_due = True
        if (seen_now and r["level"] in ("watch", "suspicious") or _watching) and _vlm_due:   # 로봇 관측 중이면 VLM도 요청
            p._vlm_t = elapsed
            ipath = None
            if latest_frame[0] is not None:
                try:
                    import cv2 as _cvq
                    _evn[0] += 1; ipath = EVID_DIR + "/req_%s.png" % p.id
                    _cvq.imwrite(ipath, latest_frame[0])
                except Exception: pass
            request_vlm(p, ipath)
        # 구역 게이트: 공개·로이터링 허용 구역의 '단순 체류'는 수상 아님(통행 정상).
        # perimeter/restricted 이거나 강한 행동(도주/반복방문)일 때만 수상 인정.
        zone_ok = sig["loitering_allowed"] and not (sig["restricted_location"] or sig["near_restricted_area"])
        strong = sig.get("movement_pattern", "") in ("run", "running", "flee") or sig.get("repeat_visits", 0) >= 3
        # 판정: 민감구역(perimeter/restricted)/강한행동 맥락에서 — 체류 충분 OR VLM 확정이면 수상.
        # VLM 단독 게이트였던 걸(=VLM 늦거나 점수<임계면 영영 안 뜸) 체류 fallback로 보강.
        _at = getattr(p, "_assess_t", 0.0)              # 관측 누적시간(캡 없음)
        gate = (not zone_ok) or strong                 # 공개·로이터링 구역 단순체류는 제외(통행 정상)
        # VLM을 우선 판단자로. dwell은 VLM이 늦/실패할 때만 받치는 안전망.
        dwell_says = _at >= (MIN_ASSESS + 2.0)          # 관측 오래(MIN_ASSESS+2)인데 VLM 미확정이면 체류로(무정지)
        vlm_says = (p.vlm_state == "done" and p.vlm_score >= VLM_SUSPECT)
        # 최소 MIN_ASSESS초 관측(노랑) 후에야 확정 -> 너무 빨리 빨강 되는 어색함 방지(자연스러운 '판단 중' 연출)
        suspicious = gate and (vlm_says or dwell_says) and _at >= MIN_ASSESS
        if flee_active[0] and p.behavior == "flee":
            suspicious = True       # 도주 중 수상자는 계속 수상 유지
        if suspicious:
            p._latched = True       # 한 번 수상으로 확정되면 시나리오 끝(reconfigure)까지 유지
        elif getattr(p, "_latched", False):
            suspicious = True
        p.suspicious = suspicious
        p._sensitive = _sensitive
        # 평가중(노랑): '카메라에 현재 인식된' 사람만 & 미확정 & (아직 VLM 판정 전 또는 민감구역). 판정 끝난 공개구역 정상인은 CLEAR.
        p.assessing = (_gap < 1.2 and not suspicious and (p.vlm_state != "done" or _sensitive))   # 1.2초 창=깜빡임 방지
        fire = suspicious and not getattr(p, "_susp_prev", False)   # normal->suspicious 1회 전이
        p._susp_prev = suspicious
        if fire:
            record_event(p, elapsed)        # 증거 캡처 + 구조화 이벤트
            if not flee_active[0]:          # 발각 -> 수상자 도주 + 차단 배정 + 발각로봇 추격
                flee_active[0] = True
                start_flee(p)
                _pid = observed_by.get(p.id, robots[0].id)
                pursuer = next((rb for rb in robots if rb.id == _pid), robots[0])
                _plan = dispatch_interception(p, pursuer)
                db_intercept(_plan, p.id, elapsed)      # 차단 결과 DB 적재
                pursuer.go_pursue(p)        # 발각한 로봇이 추격(나머지가 출구 차단)

    process_vlm(elapsed)   # 비동기 VLM 판정 도착 처리
    dbg(elapsed)           # 진단(헤드리스 포함): /tmp/dt_debug.txt

    if not HEADLESS and frame_idx % 30 == 0:
        nsus = sum(1 for p in people if p.suspicious)
        print("[%6.1fs] mode=%s rpos=(%.0f,%.0f) sus=%d/%d obs=%s" % (
            elapsed, robot.mode, robot.x, robot.z, nsus, len(people), observed_id[0]))

    # in-Isaac HUD + 3D 박스
    if elapsed - _last_state[0] > 0.3:
        _last_state[0] = elapsed
        if not HEADLESS:
            try: update_hud(elapsed)
            except Exception: pass
    if not HEADLESS:
        draw_boxes()
        if LABEL3D.get("main_txt") is not None or LABEL3D.get("pov_txt") is not None:   # 머리 위 3D 라벨(main+POV)
            try:
                from omni.ui import scene as _sc
                tp = next((p for p in people if p.suspicious), None); _st = "sus"
                if tp is None:
                    tp = next((p for p in people if getattr(p, "assessing", False)), None); _st = "asg"
                if tp is None:   # 판정 끝난 공개구역 정상인(아직 관찰 중) -> CLEAR
                    tp = next((p for p in people if p.vlm_state == "done" and not p.suspicious
                               and not getattr(p, "_sensitive", False)
                               and (elapsed - cam_seen_t.get(p.id, -9.0)) < 0.5), None); _st = "clr"
                if tp is not None:
                    _m = _sc.Matrix44.get_translation_matrix(tp.x, PERSON_Y + 240.0, tp.z)
                    if _st == "sus":
                        _txt = "SUSPICIOUS %.2f - %s" % (getattr(tp,"vlm_score",0.0), getattr(tp,"vlm_reason","") or ""); _col = _oui_color(1.0,0.15,0.15)
                    elif _st == "asg":
                        _txt = "assessing..."; _col = _oui_color(1.0,0.8,0.0)
                    else:
                        _txt = "CLEAR"; _col = _oui_color(0.2,1.0,0.35)
                for key in ("main", "pov"):
                    t = LABEL3D.get(key+"_txt"); xf = LABEL3D.get(key+"_xf")
                    if t is None: continue
                    if tp is not None:
                        xf.transform = _m; t.text = _txt; t.color = _col
                    else:
                        t.text = ""
            except Exception: pass
        if LABEL3D.get("rb_sv") is not None:   # 로봇 머리 위 라벨 위치 갱신(로봇 따라다님)
            try:
                from omni.ui import scene as _sc3
                for ri, rb in enumerate(robots):
                    xf = LABEL3D.get("rb_xf_%d" % ri)
                    if xf is not None:
                        xf.transform = _sc3.Matrix44.get_translation_matrix(rb.x, FIXED_Y + 200.0, rb.z)
            except Exception: pass
    if RUN_SECONDS and elapsed > RUN_SECONDS: break

print(">>> done. events:", len(event_log))
_logf.close()
simulation_app.close()
