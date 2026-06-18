================================================================
GIST X+AI 디지털 트윈 — 다중 로봇 데이터센터 침입 차단 데모
ReadMe : 실행 환경 / 버전 정보 (제출용)
================================================================

[1] Omniverse / Isaac Sim 앱 버전
- NVIDIA Isaac Sim 5.1.0
  (정확 빌드: 5.1.0-rc.19+release.26219.9c81211b)
- 플랫폼: Linux aarch64 (ARM64), DGX Spark / GB10
- 실행 방식: standalone SimulationApp
    <isaac>/python.sh dt_demo.py

[2] 사용한 Omniverse / Isaac 확장(Extension)
    ※ 아래 확장은 전부 Isaac Sim 5.1.0에 기본 내장 -> 별도 설치/제출 불필요.
       (확장 버전은 Isaac Sim 5.1.0 릴리스에 고정됨)
- omni.anim.people              : 보행 캐릭터(Biped) 생성
- omni.anim.graph.core          : AnimationGraph (walk / idle / lookAround)
- omni.anim.navigation.core     : navmesh 경로탐색 / 위치 스냅
- omni.replicator.core          : 로봇 카메라 렌더 product + RGB annotator
- omni.ui , omni.ui.scene       : 운영자 HUD / 머리 위 3D 라벨
- omni.kit.viewport.utility     : 뷰포트 / 카메라 POV 생성·도킹
- omni.kit.window.script_editor : (발표용) 실행 중 시나리오 트리거 입력
- omni.physx                    : FOV 콘 벽 가림 레이캐스트
- isaacsim.core.api / .utils.*  : World / stage / extensions 유틸
- isaacsim.sensors.camera       : 로봇 카메라
- isaacsim.util.debug_draw      : 구역·경로·FOV 콘 시각화
- omni.usd , pxr (USD) , carb , omni.timeline

[3] 추가 Python 모듈 (Isaac 기본에 없음 -> 반드시 설치)
- numpy == 1.26.0               (반드시 1.26 ; 타 버전 시 호환 문제)
- opencv-python-headless == 4.11.0.86
- mediapipe == 0.10.18          (사람 pose 탐지 = 지각 AI)
  설치(온라인):
    <isaac>/python.sh -m pip install -r requirements.txt
  설치(오프라인, deps/ 폴더에 wheel 동봉 시):
    <isaac>/python.sh -m pip install --no-index --find-links deps -r requirements.txt
  ※ mediapipe / opencv wheel 은 aarch64 전용. 채점 PC도 ARM64(aarch64) 필요.

[4] (선택) OpenAI API 키 — 실제 VLM(GPT-4o-mini) 판정용
- projects/.openai_key 파일에 키 한 줄. 없으면 규칙기반 mock 으로 동작(데모 정상).
- 보안상 제출 zip 에는 .openai_key 미포함.

[5] 포함된 에셋 (USD) — 실행 필수, zip 에 포함
- S7_navmesh_ready.usd  : 건물 + navmesh (63MB)
- Biped_Setup.usd       : 보행 캐릭터(AnimGraph) (49MB)
  ※ Walking.usd(정적메시 폴백)는 기본 실행에 불필요 -> 미포함(코드가 부재 시 자동 스킵)

[6] 핵심 소스
- dt_demo.py        : 메인 (시뮬 + 다중로봇 + 지각/판단/차단 통합 + 시각화/HUD/POV)
- perception.py     : mediapipe 사람 탐지
- suspicion.py      : 구역·체류·행동 신호 융합 엔진
- interception.py   : 다중로봇 출구 차단 배정 (navmesh 거리)
- zones.json        : 구역/출구 메타데이터

[7] 실행·조작 상세
- HOW_TO_RUN.md 참조 (실행 / 시나리오 트리거 / 환경변수 / zip 이관 / 녹화)
================================================================
