# Robot Patrol — Datacenter Intrusion Defense

> 다중 로봇 협동 순찰·차단 데모  
> NVIDIA Isaac Sim 5.1 · MediaPipe Pose · OpenAI GPT-4o-mini Vision

![demo screenshot](demo_screenshot.png)

건물에서 Husky 로봇 **3대**가 자율 순찰하다, 침입자가 데이터센터 경계에 잠입하면 가장 가까운 카메라 로봇이 출동·관측하고 AI가 수상 여부를 판정합니다. 침입자가 도주하면 나머지 두 로봇이 출구 2곳을 봉쇄해 검거합니다. 모든 시뮬레이션은 단일 Isaac Sim 창 안에서 동작합니다.

---

## Demo video

| Normal patrol | Intrusion & interception |
|---|---|
| [`demo_normal_v2.mp4`](demo_normal_v2.mp4) | [`demo_intrusion_v2.mp4`](demo_intrusion_v2.mp4) |

---

## Features

- **다중 로봇 협동** — Husky 3대 (R0 카메라 순찰/추격, R1 정문 차단, R2 동문 차단)
- **신호 융합 판단 엔진** — 6-channel weighted sum of `dwell` · `zone` · `behavior` · `time` · `face` · `vlm`
- **VLM in-the-loop** — GPT-4o-mini Vision API로 장면 단위 위험도 점수화
- **NavMesh 경로 계획** — Omniverse navmesh 기반 자율 보행 / 경로탐색
- **출구 차단 배정** — Greedy ETA 비교로 로봇 ↔ 출구 매칭, 도주 경로 막히면 자동 REROUTE
- **PhysX 레이캐스트 가림 처리** — 벽 너머는 보지 못함 (선택)
- **운영자 HUD** — Isaac 창 내부에 이벤트 로그 + 3대 POV 카메라 + FOV 콘 + 머리 위 ID 라벨

---

## System Architecture

```
                ┌──────────────────────────────────────────┐
                │           Isaac Sim 5.1 (DGX Spark)      │
                │                                          │
   USD scene ──▶│  NavMesh ──▶  3 × Husky (R0/R1/R2)      │
   (S7_navmesh) │                   │                      │
                │                   ▼                      │
                │           RGB camera (per robot)         │
                └──────────────────┬───────────────────────┘
                                   │  frames
                                   ▼
        ┌─────────────────────────────────────────────┐
        │  perception.py   (MediaPipe Pose)           │
        │  └─ bbox · visibility · face occlusion      │
        └─────────────────┬───────────────────────────┘
                          │  detection + zone signals
                          ▼
        ┌─────────────────────────────────────────────┐
        │  suspicion.py   (SuspicionEngine)           │
        │  └─ dwell · zone · behavior · time · face · │
        │     vlm  → level: normal / watch /          │
        │     suspicious                              │
        └─────────────────┬───────────────────────────┘
                          │ on suspicious
                          ▼
        ┌─────────────────────────────────────────────┐
        │  GPT-4o-mini Vision (async, daemon thread)  │
        │  └─ SCORE 0..1 + one-line reason            │
        └─────────────────┬───────────────────────────┘
                          │ if fleeing
                          ▼
        ┌─────────────────────────────────────────────┐
        │  interception.py   (plan_interception)      │
        │  └─ Greedy ETA assignment over exits        │
        │     {block, pursue} roles                   │
        └─────────────────────────────────────────────┘
                          │
                          ▼
                  events.db + evidence/*.png
```

---

## Modules

| File | Role |
|---|---|
| `dt_demo.py` | 메인 — Isaac Sim bootstrap, 로봇/사람 시뮬, HUD, VLM 호출, 시나리오 루프 |
| `suspicion.py` | `SuspicionEngine` — 6채널 가중합 판단, level 전이, fire 트리거 |
| `interception.py` | `plan_interception()` — 로봇·출구 greedy 매칭, suspect ETA 예측, REROUTE 감지 |
| `perception.py` | `PersonPerception` — MediaPipe Pose 1인 탐지, bbox·visibility·occlusion 신호 |
| `zones.json` | 구역(datacenter_core / perimeter / office × 2 / public)과 출구(E_MAIN, E_EAST) 메타데이터 |
| `S7_navmesh_ready.usd` | 건물 + navmesh (63 MB) |
| `Biped_Setup.usd` | AnimationGraph 보행 캐릭터 (49 MB) |
| `Walking.usd` | 정적 메시 폴백 (3 MB, 기본 실행에 불필요) |
| `record_demo.sh` | ffmpeg 화면 녹화 헬퍼 |

---

## Setup

### 1. Isaac Sim 5.1 설치 확인
- 정확 빌드: `5.1.0-rc.19+release.26219.9c81211b`
- 플랫폼: **Linux aarch64 (ARM64)** — DGX Spark / GB10 권장

### 2. Python 의존성 설치
Isaac Sim 내장 Python에 설치해야 합니다.

```bash
ISAAC=/home/user/Desktop/isaacsim          # python.sh 가 있는 폴더로 수정

# (A) 오프라인 — 동봉된 deps/ wheel 사용 (권장)
$ISAAC/python.sh -m pip install --no-index --find-links deps -r requirements.txt

# (B) 온라인
$ISAAC/python.sh -m pip install -r requirements.txt

# 설치 확인
$ISAAC/python.sh -c "import mediapipe, cv2, numpy; \
    print(mediapipe.__version__, cv2.__version__, numpy.__version__)"
```

**requirements.txt**
```
numpy==1.26.0
opencv-python-headless==4.11.0.86
mediapipe==0.10.18
```

> ⚠️ `numpy`는 반드시 **1.26** (타 버전 시 호환 문제).  
> ⚠️ mediapipe / opencv wheel은 **aarch64 (ARM64) / py3.11** 전용.

### 3. (선택) OpenAI API 키
실제 GPT-4o-mini 판정을 사용하려면 프로젝트 루트에 `.openai_key` 파일을 둡니다.

```bash
echo "sk-..." > .openai_key      # 키 한 줄 (따옴표/공백 없이)
```

키가 없으면 규칙기반 mock으로 동작하며, 데모는 정상적으로 진행됩니다.  
부팅 로그에서 `>>> VLM: OpenAI connected (gpt-4o-mini)` 메시지로 인식 여부를 확인하세요.

---

## Run

```bash
DEMO_HEADLESS=0 /home/user/Desktop/isaacsim/python.sh dt_demo.py
```

- 부팅 시간: navmesh 베이크 + Husky 3대 복제 + 카메라 3대 설정으로 **약 1.5–2 분**
- 부팅이 끝나면 Isaac Sim 창 한 개에 다음이 모두 들어옵니다:
  - **좌측** — 로봇 POV 3개(세로) + **DT OPERATOR** HUD(이벤트 로그 · AI 판정)
  - **중앙** — 3D 씬: 구역 바닥(🟥 core / 🟧 perimeter / 🟦 office), 출구 기둥, FOV 콘, 머리 위 ROBOT 1/2/3 라벨
  - **우측** — Script Editor (시나리오 트리거 입력)
- **사람 박스 색**: 🟢 정상 → 🟡 평가중(assessing) → 🔴 수상 확정(suspicious)

### Scenario 흐름
1. **NORMAL** — 정상인 3명이 각자 경로 보행, 아무도 발각되지 않음 (초록)
2. **INTRUSION** — 침입자가 복도에서 데이터센터 앞으로 잠입(빠른 걸음), 경계에서 두리번(casing)
3. 가장 가까운 카메라 로봇(R0) 출동, 마주보고 관측 → 노랑(assessing) ~4초 → **AI 판정 빨강(SUSPICIOUS)**
4. 침입자가 가장 가까운 출구로 도주 → 그 출구가 막혔으면 **반대 출구로 재도주(REROUTE)**
5. R1·R2가 두 출구(E_MAIN / E_EAST)를 봉쇄, R0가 추격
6. 출구 앞에서 검거(**CORNERED**) → 차단 성공
7. 이벤트는 `events.db`(SQLite)와 `evidence/*.png` 스냅샷으로 기록

### Manual scenario trigger
자동 순환 대신 시나리오를 직접 전환할 수 있습니다.

**Isaac 우측 Script Editor에서:**
```python
dt_scenario("intrusion")   # 침입 시나리오 시작
dt_scenario("normal")      # 정상으로 복귀
```

**또는 터미널에서:**
```bash
echo intrusion > /tmp/dt_cmd
```

---

## Suspicion Engine

`suspicion.py`의 `SuspicionEngine`은 6개 신호의 가중합으로 0–1 점수를 계산합니다.

| Signal | Weight | Source |
|---|---:|---|
| `dwell` (연속 포착 시간) | **0.52** | perception |
| `zone` (구역 위험도) | **0.22** | zones.json |
| `behavior` (행동 패턴) | **0.16** | dt_demo (idle/pace/wander/loop/run) |
| `time` (after-hours) | **0.08** | zones.json `demo_clock` |
| `face` (얼굴 가림) | **0.06** | perception (visibility < 0.25) |
| `vlm` (GPT-4o-mini) | **0.00**\* | OpenAI API |

\* VLM은 엔진 가중치 0이지만, `dt_demo.py`에서 별도 게이팅(`VLM_SUSPECT=0.5`)으로 직접 판정에 반영됩니다.

**Level 전이:** `normal → watch (seen_time ≥ 1.0s 또는 score ≥ 0.35) → suspicious (seen_time ≥ 2.5s 또는 score ≥ 0.78 또는 hard context)`

**Hard context** (즉시 suspicious): restricted 구역 내부, 또는 sensitive_perimeter에서 zone-별 loitering 임계 초과.

---

## Robot Roles

| Robot | Role | Patrol target |
|---|---|---|
| **R0** | 카메라 순찰 / 추격 (POV 1) | 복도 중앙 → datacenter 접근 |
| **R1** | 차단 (POV 2) | E_MAIN (정문) 주변 |
| **R2** | 차단 (POV 3) | E_EAST (서쪽문) 주변 |

---

## Project Structure

```
robot_patrol/
├── dt_demo.py              # main: Isaac Sim + 3 robots + HUD + VLM
├── perception.py           # MediaPipe pose detection
├── suspicion.py            # 6-channel suspicion engine
├── interception.py         # multi-robot exit-blocking planner
├── zones.json              # zones & exits metadata
├── requirements.txt
├── S7_navmesh_ready.usd    # building + navmesh (63 MB)
├── Biped_Setup.usd         # walking biped (49 MB)
├── Walking.usd             # static-mesh fallback (3 MB, optional)
├── record_demo.sh          # ffmpeg screen recorder
├── demo_normal_v2.mp4      # demo recording (normal patrol)
├── demo_intrusion_v2.mp4   # demo recording (intrusion & interception)
├── demo_screenshot.png
├── ReadMe.txt              # 제출용 환경/버전 정보
└── HOW_TO_RUN.md           # 자세한 실행 매뉴얼
```

---

## Isaac Sim Extensions (built-in to 5.1.0)

별도 설치 불필요 — Isaac Sim 5.1.0 릴리스에 모두 내장.

- `omni.anim.people` — 보행 캐릭터(Biped) 생성
- `omni.anim.graph.core` — AnimationGraph (walk / idle / lookAround)
- `omni.anim.navigation.core` — navmesh 경로탐색 / 위치 스냅
- `omni.replicator.core` — 로봇 카메라 렌더 product + RGB annotator
- `omni.ui`, `omni.ui.scene` — 운영자 HUD / 머리 위 3D 라벨
- `omni.kit.viewport.utility` — 뷰포트 / 카메라 POV 생성·도킹
- `omni.kit.window.script_editor` — 실행 중 시나리오 트리거 입력
- `omni.physx` — FOV 콘 벽 가림 레이캐스트
- `isaacsim.core.api / .utils.*` — World / stage / extensions 유틸
- `isaacsim.sensors.camera` — 로봇 카메라
- `isaacsim.util.debug_draw` — 구역·경로·FOV 콘 시각화

---

## Documentation

- [`HOW_TO_RUN.md`](HOW_TO_RUN.md) — 자세한 실행 매뉴얼
- [`ReadMe.txt`](ReadMe.txt) — 환경 / 버전 정보
