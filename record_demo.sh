#!/bin/bash
# 데모 GUI를 띄우고 부팅 완료(>>> 데모 시작) 시점에 화면 전체를 녹화.
# 결과: demo_recording.mp4 (도킹된 POV+HUD+메인 뷰포트가 한 화면에).
set -u
cd "$(dirname "$(readlink -f "$0")")"   # 스크립트가 있는 폴더(projects)로
DEMO_ONLY="${DEMO_ONLY:-}"               # 빈값=순환, "normal"/"intrusion"=해당 시나리오만
_tag="${DEMO_ONLY:-cycle}"
OUT="$(pwd)/demo_${_tag}_v2.mp4"         # 기존(demo_normal/intrusion.mp4)과 구분 -> _v2
ISAAC_PY=${ISAAC_PY:-/home/user/Desktop/isaacsim/python.sh}   # Isaac 위치 다르면 ISAAC_PY로 지정
REC_SECS=${REC_SECS:-135}

pkill -9 -f dt_demo.py 2>/dev/null; sleep 1
rm -f /tmp/guiREC.log "$OUT"

export DEMO_HEADLESS=0 DEMO_SECONDS=340 DEMO_ONLY    # export -> 데모 프로세스에 확실히 전달
"$ISAAC_PY" dt_demo.py > /tmp/guiREC.log 2>&1 &
DEMOPID=$!
echo "demo pid=$DEMOPID"

# 씬 시작 감지: patrol_log에 SCENARIO 기록되면 시나리오 루프 시작(stdout 버퍼와 무관, 최대 ~300s)
rm -f patrol_log.txt
START=0
for i in $(seq 1 150); do
  if grep -q "SCENARIO" patrol_log.txt 2>/dev/null; then START=1; echo "scene started ~$((i*2))s"; break; fi
  sleep 2
done
[ "$START" = 0 ] && echo "WARN: scene start not detected, proceeding"

sleep 3   # 첫 시나리오 렌더 안정화
WID=$(xdotool search --name "Isaac Sim Full" 2>/dev/null | head -1)
echo "WID=$WID"
if [ -n "$WID" ]; then
  wmctrl -i -r "$WID" -b add,maximized_vert,maximized_horz 2>/dev/null
  wmctrl -i -a "$WID" 2>/dev/null
  xdotool windowactivate "$WID" 2>/dev/null
fi
sleep 2

# 캡처 영역: 전체 화면(해상도 자동 감지). 일부만 원하면 CW/CH/CX/CY 환경변수로 지정.
GEO=$(xdotool getdisplaygeometry 2>/dev/null)   # 예: "1920 1080"
SW=${GEO% *}; SH=${GEO#* }
CW=${CW:-${SW:-1920}}; CH=${CH:-${SH:-1080}}; CX=${CX:-0}; CY=${CY:-0}
# x264는 짝수 해상도 필요 -> 홀수면 -1 보정
CW=$((CW - CW%2)); CH=$((CH - CH%2))
echo "REC start ${REC_SECS}s FULLSCREEN ${CW}x${CH}+${CX},${CY} @ $(date +%T)"
ffmpeg -y -f x11grab -framerate 30 -video_size "${CW}x${CH}" -i ":0.0+${CX},${CY}" -t "$REC_SECS" \
  -c:v libx264 -preset veryfast -pix_fmt yuv420p -movflags +faststart \
  "$OUT" > /tmp/ffrec.log 2>&1
echo "REC done exit=$? @ $(date +%T)"

sleep 2
kill -9 $DEMOPID 2>/dev/null; pkill -9 -f dt_demo.py 2>/dev/null
echo "=== events ==="; grep -E "SCENARIO|SUSPICIOUS|FLEE|CORNERED|INTERCEPT" patrol_log.txt 2>/dev/null | tail -12
ls -la "$OUT" 2>/dev/null
