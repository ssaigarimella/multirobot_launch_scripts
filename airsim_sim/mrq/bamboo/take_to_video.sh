#!/usr/bin/env bash
# One bamboo flight take -> one finished video in Lucas's locked format.
#   title -> MRQ cinematic in SwarmVideo/BambooGrove -> "one shared map" card
#   -> full-screen merged-map RViz -> end title.
#
# Every stage has a fallback so the overnight loop can call this unattended on
# ANY take, however badly the fleet flew:
#   * window_select --relaxed  never refuses (shot_qc is the real gate)
#   * each designed camera grammar is tried, and any beat it refuses is filled
#     by design_camera_fallback.py's searched orbit
#   * the map segment is reused if already reshot, else regenerated from the bag
#
# usage: take_to_video.sh <run_label> [settings_json]
set -u
LBL=${1:?run label}
SET=${2:-/home/lucas/hercules-sim/settings-bamboo-4drone.json}
B=/home/lucas/UE5/hercules-sim-big/mrq_work/bamboo
M=/home/lucas/UE5/hercules-sim-big/mrq_work/mapview
H=/home/lucas/hercules-sim
SP=${SCRATCH:-/tmp/bamboo_mrq}
mkdir -p "$SP"
cd "$B"

# --- world anchor chosen by pick_anchor.py against grove_harvest.json ---
# BambooGrove, outer culm ring: flat ground (z=0), 0.267 culms/m2 in the lane,
# every camera position still over the ground tiles.
ANCHOR=${ANCHOR:-5400,-2100,0}
YAWD=${YAWD:-350}
SCALE=${SCALE:-0.55}
ZLIFT=${ZLIFT:-43}
EV=${CAM_EV_BIAS:-1.5}
MAP=${MRQ_MAP_PATH:-/Game/SwarmVideo/BambooGrove}
export QC_OCC_FRAC=${QC_OCC_FRAC:-0.30}      # a grove always has stalks in the way
export MRQ_MAP_PATH=$MAP

echo "=== [$LBL] 1/8 timeline + window selection ==="
(cd $H && python3 investigation/score_run.py "$LBL" --timeline >/dev/null 2>&1)
WR=""; BEST=-1
for a in 40 55 70 85 100 120 150 180; do
  r=$SP/wr_${LBL}_$a.json
  out=$(python3 window_select.py "$LBL" --settings "$SET" --relaxed \
        --open-anchor-s $a --report "$r" 2>&1)
  echo "$out" | tail -1 | grep -qE "BLESSED|RELAXED" || continue
  sp=$(python3 -c "
import json,sys
try:
    d=json.load(open('$r'));w=d.get('w_open') or {}
    print(w.get('spread',0)+100*int(bool(w.get('feasible'))))
except Exception: print(-1)")
  ok=$(python3 -c "print(1 if float('$sp')>float('$BEST') else 0)")
  [ "$ok" = "1" ] && { BEST=$sp; WR=$r; }
done
[ -n "$WR" ] || { echo "  [$LBL] NO WINDOW AT ALL — skipping"; exit 4; }
echo "  window report $WR (open score $BEST)"

echo "=== [$LBL] 2/8 keys -> bamboo grove ==="
MRQ_ANCHOR="1370,1340,100" python3 ../ue/export_keys.py \
  $H/e1_frames/runs/$LBL "$SET" keys_${LBL}_blocks.json 999 >/dev/null || exit 5
TEMPLE_ANCHOR="$ANCHOR" YAW_DEG=$YAWD SCALE=$SCALE ZLIFT=$ZLIFT \
  python3 transform_keys_temple.py keys_${LBL}_blocks.json keys_${LBL}_grove_raw.json >/dev/null || exit 5
ITERS=120 MAX_OFF=250 python3 clearance_bamboo.py \
  keys_${LBL}_grove_raw.json keys_${LBL}_grove_full.json > $SP/${LBL}_clearance.log 2>&1
grep -E "SWEEP|max lateral" $SP/${LBL}_clearance.log
[ -f keys_${LBL}_grove_full.json ] || cp keys_${LBL}_grove_raw.json keys_${LBL}_grove_full.json
python3 make_shot_keys_v6.py keys_${LBL}_grove_full.json "$WR" keys_${LBL}_base.json >/dev/null || exit 5

echo "=== [$LBL] 3/8 cameras (designed grammars, orbit fallback for refusals) ==="
cp keys_${LBL}_base.json cur_${LBL}.json
if S1_SEP_MIN=150 python3 design_camera_opening.py cur_${LBL}.json nxt_${LBL}.json \
     > $SP/${LBL}_s1s2.log 2>&1 && [ -f nxt_${LBL}.json ]; then
  mv nxt_${LBL}.json cur_${LBL}.json; echo "  S1+S2 designed"
elif S1_SEP_MIN=150 S2_SKIP=1 python3 design_camera_opening.py cur_${LBL}.json nxt_${LBL}.json \
     > $SP/${LBL}_s1s2b.log 2>&1 && [ -f nxt_${LBL}.json ]; then
  mv nxt_${LBL}.json cur_${LBL}.json; echo "  S1 designed (S2 skipped)"
else
  echo "  S1/S2 refused -> orbit fallback"
fi
rm -f nxt_${LBL}.json
if python3 design_camera_track.py cur_${LBL}.json nxt_${LBL}.json > $SP/${LBL}_s3.log 2>&1 \
   || V2FB=1 python3 design_camera_track.py cur_${LBL}.json nxt_${LBL}.json > $SP/${LBL}_s3b.log 2>&1; then
  [ -f nxt_${LBL}.json ] && { mv nxt_${LBL}.json cur_${LBL}.json; echo "  S3 designed"; }
fi
rm -f nxt_${LBL}.json
if V6=1 python3 design_camera_v3.py cur_${LBL}.json nxt_${LBL}.json > $SP/${LBL}_s4.log 2>&1; then
  [ -f nxt_${LBL}.json ] && { mv nxt_${LBL}.json cur_${LBL}.json; echo "  S4 designed"; }
fi
rm -f nxt_${LBL}.json
KEYS=$B/keys_${LBL}_grove_full.json OUT_DIR=$B python3 bake_ribbons_v6.py > $SP/${LBL}_ribbons.log 2>&1
if FULL_KEYS=$B/keys_${LBL}_grove_full.json python3 design_camera_arc.py \
     cur_${LBL}.json nxt_${LBL}.json > $SP/${LBL}_s5s6.log 2>&1; then
  [ -f nxt_${LBL}.json ] && { mv nxt_${LBL}.json cur_${LBL}.json; echo "  S5+S6 designed"; }
elif KEYS_IN=cur_${LBL}.json KEYS_OUT=nxt_${LBL}.json python3 s5_from_s6.py > $SP/${LBL}_s5fb.log 2>&1; then
  [ -f nxt_${LBL}.json ] && { mv nxt_${LBL}.json cur_${LBL}.json; echo "  S5 from S6 vantage"; }
fi
rm -f nxt_${LBL}.json
python3 design_camera_fallback.py cur_${LBL}.json keys_${LBL}_final.json \
  > $SP/${LBL}_fallback.log 2>&1 || true
[ -f keys_${LBL}_final.json ] || cp cur_${LBL}.json keys_${LBL}_final.json
grep -E "QC |already designed" $SP/${LBL}_fallback.log || true

echo "=== [$LBL] 4/8 wait for a flight-loop gap ==="
for i in $(seq 1 240); do
  pgrep -f "run_exp.sh|capture_fleet_bagonly.sh" >/dev/null || break
  sleep 30
done

echo "=== [$LBL] 5/8 MRQ render (BambooGrove, ~45 min) ==="
rm -rf out/${LBL}
S5A=$(python3 -c "
import json;K=json.load(open('keys_${LBL}_final.json'))
s=[x for x in K['info']['shots'] if x['name']=='s5_ribbonarc']
print(f\"{s[0]['start']}:{s[0]['start']+60}\" if s else '0:1')")
CAM_EV_BIAS=$EV RIBBONS=1 RIBBON_REVEAL=$S5A ./render_fleet_bamboo.sh \
  $B/keys_${LBL}_final.json $B/out/${LBL} 0 > $SP/${LBL}_render.log 2>&1
grep -q RENDER_OK $SP/${LBL}_render.log || { echo "  [$LBL] render failed"; tail -5 $SP/${LBL}_render.log; exit 8; }

echo "=== [$LBL] 6/8 map segment ==="
cd $M
if [ ! -f reshot_${LBL}_map.mkv ]; then
  [ -d bag_${LBL}_shaded ] || python3 retint_bag.py bag_${LBL} bag_${LBL}_shaded > $SP/${LBL}_retint.log 2>&1
  BAGD=bag_${LBL}_shaded; [ -d "$BAGD" ] || BAGD=bag_${LBL}
  BAG=$M/$BAGD RVIZ_CFG=$M/fleet_map_bamboo.rviz \
    AUG_ARGS="--spawn-ned 1:0.0:-4.5,2:0.0:-1.5,3:0.0:1.5,4:0.0:4.5" \
    ./reshot_from_bag.sh ${LBL}_map 1 1 > $SP/${LBL}_reshot.log 2>&1
fi
BAGD=bag_${LBL}_shaded; [ -d "$M/$BAGD" ] || BAGD=bag_${LBL}

echo "=== [$LBL] 7/8 compose ==="
cd $B
LABEL=$LBL KEYS=$B/keys_${LBL}_final.json FRAMES=$B/out/${LBL} \
  RESHOT=$M/reshot_${LBL}_map.mkv GRABSTART=$M/reshot_${LBL}_map_grabstart.txt \
  PLAYSTART=$M/reshot_${LBL}_map_playstart.txt BAG=$M/$BAGD RATE=1 \
  ./compose_final_bamboo.sh > $SP/${LBL}_compose.log 2>&1
tail -3 $SP/${LBL}_compose.log

echo "=== [$LBL] 8/8 result ==="
ls -la $H/bamboo_full_${LBL}*.mp4 2>/dev/null && echo "=== [$LBL] VIDEO_OK ===" \
  || { echo "=== [$LBL] compose failed ==="; exit 9; }
