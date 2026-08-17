#!/usr/bin/env bash
# MRQ pipeline for the BAMBOO FOREST cinematic (UE 5.6, SwarmReplay project).
# Stage A: UnrealEditor-Cmd -ExecutePythonScript=build_fleet_seq6_bamboo.py (nullrhi)
# Stage B: UnrealEditor-Cmd MoviePipelineEntryMap -game -MoviePipelineConfig=...
#          (hercules stage-B manifest bootstrap, proven in 5.2; kungfu MRQ
#           settings are baked into the manifest by stage A.)
# Honors the kungfu /tmp/ue_mrq.lock protocol (one MRQ render per GPU).
#
# usage: render_fleet_temple.sh <keys.json> <out_frames_dir> [TEST] [TEST_START] [TEST_END]
set -e
KEYS=${1:?keys json}
OUT=${2:?frames out dir}
TEST=${3:-0}
TEST_START=${4:-600}
TEST_END=${5:-615}

UE=/home/lucas/UE5/UE5.6/Engine/Binaries/Linux
PROJ="/home/lucas/UE5/educationalVideos/drones/swarm/ue5/SwarmReplay"
W=/home/lucas/UE5/hercules-sim-big/mrq_work/bamboo
LOCK=/tmp/ue_mrq.lock

mkdir -p "$OUT"

echo "=== STAGE A: bake sequence + manifest ==="
rc=1
for attempt in 1 2 3; do
  rm -f "$W/stageA_clean_progress.txt"
  KEYS_JSON=$KEYS OUT_DIR=$OUT TEST=$TEST TEST_START=$TEST_START TEST_END=$TEST_END \
  "$UE/UnrealEditor-Cmd" "$PROJ/SwarmReplay.uproject" \
    -ExecutePythonScript="$W/build_fleet_seq6_bamboo.py" \
    -nullrhi -stdout -unattended -nosplash > "$W/stageA_clean.log" 2>&1 || true
  if grep -q "BUILD_OK" "$W/stageA_clean_progress.txt" 2>/dev/null; then rc=0; break; fi
  rc=1
  echo "  stage A attempt $attempt failed - retrying (UE cold-start segfaults possible)"
  sleep 5
done
cat "$W/stageA_clean_progress.txt" 2>/dev/null
grep -E "LogPython: Error" "$W/stageA_clean.log" | tail -15
[ $rc -eq 0 ] || { echo "STAGE A FAILED"; exit 1; }

echo "=== acquire MRQ lock ==="
while true; do
  if [ -f "$LOCK" ]; then
    age=$(( $(date +%s) - $(stat -c %Y "$LOCK") ))
    if [ "$age" -gt 1200 ]; then rm -f "$LOCK"; else sleep 10; continue; fi
  fi
  echo "hercules-bamboo-agent $(date +%H:%M:%S)" > "$LOCK" && break
done
trap 'rm -f "$LOCK"' EXIT

echo "=== STAGE B: MRQ render ==="
rc=1
for attempt in 1 2 3; do
  "$UE/UnrealEditor-Cmd" "$PROJ/SwarmReplay.uproject" MoviePipelineEntryMap \
    -game -MoviePipelineConfig="MovieRenderPipeline/QueueManifest.utxt" \
    -RenderOffscreen -windowed -ResX=1920 -ResY=1080 \
    -log -stdout -unattended -nosplash -notexturestreaming > "$W/stageB_clean.log" 2>&1 || true
  n=$(ls "$OUT"/frame_*.png 2>/dev/null | wc -l)
  echo "  stage B attempt $attempt frames=$n"
  [ "$n" -gt 0 ] && { rc=0; break; }
  rc=1
  sleep 5
done
grep -E "LogMovieRenderPipeline|Error:" "$W/stageB_clean.log" | tail -25
[ $rc -eq 0 ] || { echo "STAGE B FAILED"; exit 1; }
echo "RENDER_OK $(ls "$OUT"/frame_*.png | wc -l) frames in $OUT"
