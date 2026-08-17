#!/usr/bin/env bash
# FINAL composite for the BAMBOO FOREST cinematic — Lucas's locked format:
#   title -> cinematic beats (MRQ frames, SwarmVideo/BambooGrove) -> "one shared
#   map" card -> full-screen merged-map RViz (growth + final hold) -> end title.
# Card colours, honesty label, amber speed chips, per-drone legend colours and
# the <5MB 960x540 mobile variant all follow compose_final_temple_c13d.sh.
#
# env: KEYS (final stitched keys json), FRAMES (MRQ frames dir),
#      RESHOT (reshot mkv), GRABSTART/PLAYSTART (epoch files), BAG (bag dir),
#      RATE (bag play rate, default 1), LABEL (run label), OUT/OUTM overrides,
#      MAPCROP (ffmpeg crop for the RViz 3D panel)
set -eu
W=/home/lucas/UE5/hercules-sim-big/mrq_work/bamboo
M=/home/lucas/UE5/hercules-sim-big/mrq_work/mapview
LABEL=${LABEL:?run label}
C=$W/composite_$LABEL
FONT=/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf
KEYS=${KEYS:?final keys json}
FRAMES=${FRAMES:?frames dir}
RESHOT=${RESHOT:?reshot mkv}
GRABSTART=${GRABSTART:?grabstart file}
PLAYSTART=${PLAYSTART:?playstart file}
BAG=${BAG:?bag dir}
RATE=${RATE:-1}
MAPCROP=${MAPCROP:-1450:910:376:90}
OUT=${OUT:-/home/lucas/hercules-sim/bamboo_full_${LABEL}.mp4}
OUTM=${OUTM:-/home/lucas/hercules-sim/bamboo_full_${LABEL}_mobile.mp4}
mkdir -p "$C"

# ---- shot table + clearance note from the keys json ----
mapfile -t SHOT_LINES < <(python3 - "$KEYS" <<'PYEOF'
import json, sys
K = json.load(open(sys.argv[1]))
for s in K["info"]["shots"]:
    print(f"{s['name']} {s['start']} {s['end']} {s.get('retime',1.0)}")
PYEOF
)
DODGE=$(python3 - "$KEYS" <<'PYEOF'
import json, sys
K = json.load(open(sys.argv[1]))
c = K["info"].get("bamboo_clearance") or {}
print(f"{c.get('max_off_cm', 0)/100.0:.1f}")
PYEOF
)
echo "shots:"; printf '  %s\n' "${SHOT_LINES[@]}"
echo "max clearance offset: ${DODGE} m"

# ---- first-mesh offset inside the reshot clip ----
OFF=$(python3 - "$BAG" "$GRABSTART" "$PLAYSTART" "$RATE" <<'PYEOF'
import glob, sqlite3, sys
bag, gs_f, ps_f, rate = sys.argv[1], sys.argv[2], sys.argv[3], float(sys.argv[4])
db = glob.glob(bag + "/*.db3")[0]
con = sqlite3.connect(db)
t0 = con.execute("select min(timestamp) from messages").fetchone()[0]
tm = con.execute(
    "select min(timestamp) from messages where topic_id in "
    "(select id from topics where name like '%/mesh')").fetchone()[0]
gs = float(open(gs_f).read().strip())
ps = float(open(ps_f).read().strip())
off = max(0.0, (ps - gs) + (tm - t0) / 1e9 / rate - 2.0)
print(f"{off:.2f}")
PYEOF
)
DUR=$(ffprobe -v error -show_entries format=duration -of csv=p=0 "$RESHOT")
echo "map growth starts at ${OFF}s of reshot (dur ${DUR}s)"

LABEL_TXT='HERCULES fleet - Bamboo Forest - recorded autonomy replay'
CONCAT="$C/concat.txt"; : > "$CONCAT"

# ---- title card ----
ffmpeg -y -loglevel warning -f lavfi -i "color=c=0x14161e:s=1920x1080:d=2.5:r=30" -filter_complex "
drawtext=fontfile=$FONT:text='HERCULES Fleet Exploration — 3D':fontsize=72:fontcolor=white:x=(w-tw)/2:y=h/2-118,
drawtext=fontfile=$FONT:text='Bamboo Forest - Movie Render Queue cinematic':fontsize=36:fontcolor=0xa8b0c0:x=(w-tw)/2:y=h/2-4,
drawtext=fontfile=$FONT:text='4x autonomous 3D exploration (cuVSLAM + nvblox + FIS + 3D A*) - ground-truth trajectory replay':fontsize=26:fontcolor=0x76808f:x=(w-tw)/2:y=h/2+62,
drawtext=fontfile=$FONT:text='flown paths shifted up to ${DODGE} m to clear stalks in this grove':fontsize=22:fontcolor=0x5d6673:x=(w-tw)/2:y=h/2+108" \
  -c:v libx264 -preset slow -crf 18 -pix_fmt yuv420p "$C/title.mp4"
echo "file '$C/title.mp4'" >> "$CONCAT"

# ---- world beats (one trim per shot) ----
for line in "${SHOT_LINES[@]}"; do
  read -r name start end retime <<< "$line"
  n=$((end - start))
  first=$(printf "%s/frame_%04d.png" "$FRAMES" "$start")
  [ -f "$first" ] || { echo "  skip $name (no frames)"; continue; }
  seg="$C/seg_$name.mp4"
  VF="drawtext=fontfile=$FONT:text='$LABEL_TXT':fontsize=32:fontcolor=white:borderw=2:bordercolor=black@0.7:x=28:y=24"
  is_fast=$(python3 -c "print(1 if float('$retime') > 1.05 else 0)")
  if [ "$is_fast" = "1" ]; then
    chip=$(python3 -c "print(f\"{float('$retime'):.1f}x speed\")")
    VF="$VF,drawtext=fontfile=$FONT:text='$chip':fontsize=28:fontcolor=0xffd27a:borderw=2:bordercolor=black@0.8:x=w-tw-28:y=24"
  fi
  case "$name" in
    s1_starburst)  cap='four goals claimed over the shared radio';;
    s3_track)      cap='onboard stereo + cuVSLAM, no GPS under the canopy';;
    s5_ribbonarc)  cap='flown paths - one 5-minute flight, four drones';;
    *)             cap='';;
  esac
  if [ -n "$cap" ]; then
    VF="$VF,drawtext=fontfile=$FONT:text='$cap':fontsize=26:fontcolor=0xd8e0ec:borderw=2:bordercolor=black@0.7:x=28:y=h-52"
  fi
  ffmpeg -y -loglevel warning -framerate 30 -start_number "$start" \
    -i "$FRAMES/frame_%04d.png" -frames:v "$n" -vf "$VF" \
    -c:v libx264 -preset slow -crf 18 -pix_fmt yuv420p "$seg"
  echo "file '$seg'" >> "$CONCAT"
done

# ---- transition card ----
ffmpeg -y -loglevel warning -f lavfi -i "color=c=0x14161e:s=1920x1080:d=2.2:r=30" -filter_complex "
drawtext=fontfile=$FONT:text='one shared map - live':fontsize=58:fontcolor=white:x=(w-tw)/2:y=h/2-52,
drawtext=fontfile=$FONT:text='four drones, four onboard nvblox maps, one world - captured during the flight':fontsize=27:fontcolor=0x8f98a8:x=(w-tw)/2:y=h/2+34" \
  -c:v libx264 -preset slow -crf 18 -pix_fmt yuv420p "$C/transition.mp4"
echo "file '$C/transition.mp4'" >> "$CONCAT"

# ---- merged-map segment: growth + final hold ----
LEGEND="drawtext=fontfile=$FONT:text='GHOST':fontsize=26:fontcolor=0x3f8cff:borderw=2:bordercolor=black@0.8:x=28:y=h-44,
drawtext=fontfile=$FONT:text='DELTA':fontsize=26:fontcolor=0x3fff72:borderw=2:bordercolor=black@0.8:x=160:y=h-44,
drawtext=fontfile=$FONT:text='BUCKSHEE':fontsize=26:fontcolor=0xff4cf2:borderw=2:bordercolor=black@0.8:x=292:y=h-44,
drawtext=fontfile=$FONT:text='THUNDERSTRIKE':fontsize=26:fontcolor=0xff941e:borderw=2:bordercolor=black@0.8:x=488:y=h-44"
# The reshot plays the bag at RATE; a raw 30 s trim would only show the first
# ~10% of a 5-minute flight growing.  Take a long source window and speed-ramp
# it into the 30 s slot so the beat actually shows the map filling in, with an
# honest amber speed chip carrying the total factor.
GROW=${GROW_S:-30}
read -r GROW_SRC SPEED TOTALX < <(python3 - "$DUR" "$OFF" "$GROW" "${GROW_SRC_MAX:-170}" "$RATE" <<'PYEOF'
import sys
dur, off, grow, cap, rate = (float(v) for v in sys.argv[1:6])
src = max(grow, min(dur - off - 6.0, cap))
speed = src / grow
print(f"{src:.2f} {speed:.3f} {speed*rate:.0f}")
PYEOF
)
CHIP="drawtext=fontfile=$FONT:text='${TOTALX}x speed':fontsize=28:fontcolor=0xffd27a:borderw=2:bordercolor=black@0.8:x=w-tw-28:y=24,"
MAPVF="[0:v]crop=${MAPCROP},scale=-2:1080,pad=1920:1080:(ow-iw)/2:0:color=0x0d0e14,setpts=PTS/${SPEED},
drawtext=fontfile=$FONT:text='HERCULES fleet - shared map, captured live during the run':fontsize=30:fontcolor=white:borderw=2:bordercolor=black@0.7:x=28:y=24,
$CHIP
$LEGEND,
drawtext=fontfile=$FONT:text='four onboard nvblox maps growing into one shared world':fontsize=24:fontcolor=0xaab2c0:borderw=2:bordercolor=black@0.7:x=w-tw-28:y=h-44[v]"
echo "map growth: ${GROW_SRC}s of reshot -> ${GROW}s (${SPEED}x, ${TOTALX}x vs real time)"
ffmpeg -y -loglevel warning -ss "$OFF" -t "$GROW_SRC" -i "$RESHOT" -filter_complex "$MAPVF" \
  -map "[v]" -c:v libx264 -preset slow -crf 18 -pix_fmt yuv420p "$C/map_growth.mp4"
HOLD_SS=$(python3 -c "print(max(0.0, float('$DUR') - 6.0))")
ffmpeg -y -loglevel warning -ss "$HOLD_SS" -t 5 -i "$RESHOT" -filter_complex "
[0:v]crop=${MAPCROP},scale=-2:1080,pad=1920:1080:(ow-iw)/2:0:color=0x0d0e14,
drawtext=fontfile=$FONT:text='final shared map - one 5-minute flight':fontsize=30:fontcolor=white:borderw=2:bordercolor=black@0.7:x=28:y=24,
$LEGEND[v]" \
  -map "[v]" -c:v libx264 -preset slow -crf 18 -pix_fmt yuv420p "$C/map_hold.mp4"
echo "file '$C/map_growth.mp4'" >> "$CONCAT"
echo "file '$C/map_hold.mp4'" >> "$CONCAT"

# ---- end title ----
ffmpeg -y -loglevel warning -f lavfi -i "color=c=0x14161e:s=1920x1080:d=3:r=30" -filter_complex "
drawtext=fontfile=$FONT:text='HERCULES':fontsize=84:fontcolor=white:x=(w-tw)/2:y=h/2-80,
drawtext=fontfile=$FONT:text='four autonomous drones - one flight - one shared map':fontsize=32:fontcolor=0xa8b0c0:x=(w-tw)/2:y=h/2+18" \
  -c:v libx264 -preset slow -crf 18 -pix_fmt yuv420p "$C/endtitle.mp4"
echo "file '$C/endtitle.mp4'" >> "$CONCAT"

# ---- concat + mobile ----
ffmpeg -y -loglevel warning -f concat -safe 0 -i "$CONCAT" -c copy "$OUT"
ffprobe -v error -show_entries format=duration,size -of default=nw=1 "$OUT"

crf=28
while :; do
  ffmpeg -y -loglevel warning -i "$OUT" -vf "scale=960:540" \
    -c:v libx264 -preset slow -crf $crf -pix_fmt yuv420p -movflags +faststart "$OUTM"
  sz=$(stat -c %s "$OUTM"); echo "mobile crf=$crf size=$sz"
  [ "$sz" -lt 5000000 ] && break
  crf=$((crf+2)); [ $crf -gt 40 ] && break
done
echo "COMPOSITE_BAMBOO_OK $OUT"
