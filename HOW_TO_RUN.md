# 디지털 트윈 순찰 시연 — 실행 방법 (다중 로봇 + Datacenter 침입 차단)

GIST X+AI · Husky 3대가 건물을 순찰 → 침입자가 datacenter로 잠입해 casing → 카메라 로봇 출동·AI 판정 → 도주 → 다중로봇 출구 차단(interception).

> 버전/환경 정보는 **ReadMe.txt** 참조 (Isaac Sim 5.1.0 + extension + 의존성).

---

## 0. 의존성 설치 (Isaac python 에)
Isaac Sim 5.1 의 python 에 **mediapipe / opencv / numpy 1.26** 이 있어야 perception 이 동작.
```bash
ISAAC=/home/user/Desktop/isaacsim          # python.sh 가 있는 폴더(설치 위치 다르면 수정)
# (A) 오프라인 — 동봉된 deps/ 의 wheel 사용  ★권장(외부 PC)
$ISAAC/python.sh -m pip install --no-index --find-links deps -r requirements.txt
# (B) 온라인
$ISAAC/python.sh -m pip install -r requirements.txt
# 확인
$ISAAC/python.sh -c "import mediapipe,cv2,numpy; print(mediapipe.__version__,cv2.__version__,numpy.__version__)"
```
※ deps/ wheel 은 **aarch64(ARM64)/py3.11 전용** — 채점 PC도 동일 아키텍처여야 함.
※ numpy 는 반드시 1.26.

## 0-1. (선택) OpenAI 키 — 실제 GPT-4o-mini 판정
키 없이도 동작(규칙기반 mock). 실제 VLM 판정을 쓰려면 projects 폴더에 `.openai_key`:
```bash
echo "sk-..." > .openai_key      # 키 한 줄 (따옴표/공백 없이)
```
- 부팅 로그 `>>> AI 판정: OpenAI 연결됨` = 키 인식. `.openai_key` 는 제출 zip 에서 제외.

---

## 1. 실행 (Isaac 창 하나에 다 들어감)
```bash
cd <projects 폴더>
DEMO_HEADLESS=0 /home/user/Desktop/isaacsim/python.sh dt_demo.py
```
- 부팅(navmesh 베이크 + Husky 복제 + 카메라 3대) **약 1.5~2분** → Isaac Sim 창 + 순찰 시작
- 화면 구성(웹 대시보드 없음, 전부 Isaac 창 안에):
  - **좌측**: 로봇 카메라 POV 3개(세로) + **DT OPERATOR** HUD(이벤트 로그 + AI 판정)
  - **중앙**: 3D 씬 — 구역 바닥(빨강 core / 주황 perimeter / 파랑 office), 출구 기둥, **FOV 콘**(카메라 시야), 머리 위 **ROBOT 1/2/3** 라벨
  - **우측**: Script Editor (발표용 시나리오 입력 — 아래 3번)
  - 사람 박스 색 = **초록(정상) → 노랑(평가중) → 빨강(수상 확정)**

## 2. 시나리오 흐름 (자동, 정상 ↔ 침입 번갈아)
1. **NORMAL**: 정상인 3명이 각자 경로 보행 → 아무도 발각 안 됨(초록)
2. **INTRUSION**: 침입자가 **복도에서 datacenter 앞으로 잠입(빠른 걸음)** → 경계에서 두리번(casing)
3. **가장 가까운 카메라 로봇(R0)이 출동**해 마주보고 관측 → 노랑(assessing) ~4초 → **AI 판정 빨강(SUSPICIOUS)**
4. 침입자가 **가장 가까운 출구로 도주** → 그 출구가 막혔으면 **반대 출구로 재도주(REROUTE)**
5. 로봇 2·3 이 두 출구(E_MAIN/E_EAST) 봉쇄, R0 은 추격 → **출구 앞에서 검거(CORNERED) = 차단 성공**
6. 이벤트는 `events.db`(SQLite)에 사진·위치·AI점수·차단결과로 기록 + `evidence/` 에 증거 사진

## 3. ★ 발표용 — Isaac 안에서 시나리오 직접 트리거
자동 순환 대신 **멘트에 맞춰 수동 전환** 가능. **우측 Script Editor** 에 입력하고 Run:
```python
dt_scenario("intrusion")   # 침입 시나리오 시작
dt_scenario("normal")      # 정상으로 복귀
```
- 또는 터미널에서: `echo intrusion > /tmp/dt_cmd`
- 활용: 미리 부팅 → 화면 공유 → "순찰 중입니다" 설명 → `dt_scenario("intrusion")` → 발각/차단 시연
- **Google Meet 등 라이브 시연**: 부팅(~2분)을 미리 해두고 실행된 창을 공유하면 부팅이 안 보임.

---

## 4. 시연 PC로 옮겨 실행 (zip 이관/제출) ★중요
경로는 **스크립트 위치 기준 상대경로** → projects 폴더를 어디에 풀든 동작.
1) Isaac Sim 5.1 위치 찾기: `find / -name python.sh -path "*isaac*" 2>/dev/null`
2) 의존성 설치: 위 **0번** (오프라인 deps/ 권장)
3) 실행 위치가 `/home/user/Desktop/isaacsim` 이 아니면 GUI experience kit 지정:
```bash
ISAAC=/path/to/isaacsim
DEMO_HEADLESS=0 DEMO_ISAAC_EXP=$ISAAC/apps/isaacsim.exp.full.kit $ISAAC/python.sh dt_demo.py
```

### zip 에 포함 / 제외
- **포함**: `*.py`, `zones.json`, **USD 2개**(S7_navmesh_ready 63M / Biped_Setup 49M; Walking.usd 폴백은 불필요),
  `requirements.txt`, **`deps/`(266M, 오프라인 모듈)**, `record_demo.sh`, `ReadMe.txt`, `HOW_TO_RUN.md`
- **제외**: `.openai_key`(키!), `.claude/ .codex/ .agents/ .git/`(설정), 재생성물(`evidence/ events.db cam_live.jpg patrol_log.txt __pycache__/`)
- 제출 zip 예시:
```bash
cd ~/Desktop
zip -r dt_demo_submit.zip projects \
  -x "projects/.openai_key" "projects/.claude/*" "projects/.codex/*" "projects/.agents/*" \
     "projects/.git/*" "projects/__pycache__/*" "projects/evidence/*" "projects/demo_frames/*" \
     "projects/events.db" "projects/cam_live.jpg" "projects/patrol_log.txt"
```

---

## 5. 환경변수(옵션)
- `DEMO_HEADLESS=1` : 창 없이 빠른 검증(진단 `/tmp/dt_debug.txt`)
- `DEMO_SECONDS=120` : 실행 시간 제한(0=무한)
- `DEMO_ONLY=normal|intrusion` : 한 시나리오만 반복(분리 녹화용)
- `DEMO_MULTICAM=0` : 카메라 1대만(기본 3대) — fps↑/부드러움 (1대는 dispatch로 단독 탐지)
- `DEMO_POV=0` / `DEMO_POV_ALL=0` : POV 뷰포트 끄기 / robot0 POV만
- `DEMO_CONE=0` / `DEMO_CONE_RANGE=...` : FOV 콘 끄기 / 콘 길이(기본=SENSE_RANGE)
- `DEMO_MIN_ASSESS=4` : 노랑(평가) 최소 유지 후 확정까지 시간(초)
- `DEMO_COORD=0` : 동적 커버리지 분담(R0가 문 근처면 구역로봇 안쪽) 끄기
- `DEMO_VLM_SUSPECT=0.5` / `DEMO_VLM_MODEL=gpt-4o-mini` : VLM 임계 / 모델
- `DEMO_ISAAC_EXP=...` : GUI experience kit 경로(Isaac 위치 다를 때)

## 6. 화면 녹화 (선택)
```bash
DEMO_ONLY=normal    REC_SECS=40 bash record_demo.sh   # -> demo_normal_v2.mp4
DEMO_ONLY=intrusion REC_SECS=45 bash record_demo.sh   # -> demo_intrusion_v2.mp4
ISAAC_PY=$ISAAC/python.sh bash record_demo.sh          # Isaac 위치 다를 때
```
전체화면 자동 캡처(해상도 자동). ffmpeg 필요(`sudo apt install -y ffmpeg`).

## 7. 시나리오 좌표 / 로봇 역할 (dt_demo.py 상단)
- `PATROL_ROUTE` : R0(카메라) 순찰 — 중앙→datacenter 접근(발각·추격, POV)
- `R1_DOOR/R2_DOOR` : R1=E_MAIN(정문) / R2=E_EAST(서쪽문) 출구 지킴(차단)
- `SUSPECT_ROUTE` : 침입자 잠입 경로(복도→datacenter 앞)
- `WALK_ROUTES` : 정상인 보행 경로
- 속도: 순찰 200 / 추격 300 / 도주 260 / 차단 600 (cm/s)

## 8. 구성 모듈
- `dt_demo.py` — 메인 / `perception.py` — mediapipe 탐지 / `suspicion.py` — 신호융합 / `interception.py` — 출구차단 배정 / `zones.json` — 구역·출구
