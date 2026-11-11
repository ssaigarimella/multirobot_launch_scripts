#!/bin/sh
# Mirror host USB/serial/video device nodes into the running container for
# $1 seconds (default 240; 0 = a single pass). The container's /dev is a
# snapshot taken at container start, so devices re-enumerated afterwards
# (the camera driver's initial_reset, replugged cables) are otherwise
# invisible inside it.
# One docker exec per pass: the host lists "path major minor" and the
# container creates or fixes only the nodes that are missing or differ.
end=$(( $(date +%s) + ${1:-240} ))
while :; do
  for f in /dev/bus/usb/*/* /dev/ttyUSB* /dev/video* /dev/media*; do
    [ -c "$f" ] && echo "$f $(printf %d 0x$(stat -c %t "$f")) $(printf %d 0x$(stat -c %T "$f"))"
  done | docker exec -i -u root isaac_ros_realsense sh -c '
    while read f maj min; do
      want=$(printf "%x:%x" "$maj" "$min")
      if [ ! -c "$f" ] || [ "$(stat -c %t:%T "$f")" != "$want" ]; then
        rm -f "$f"; mkdir -p "$(dirname "$f")"
        mknod -m 666 "$f" c "$maj" "$min" && echo "synced $f"
      fi
    done' 2>/dev/null
  [ "$(date +%s)" -lt "$end" ] || break
  sleep 3
done
