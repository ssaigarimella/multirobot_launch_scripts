#!/bin/sh
# Mirror host USB/serial/video device nodes into the running container for
# $1 seconds. The container's /dev is a snapshot taken at container start, so
# devices re-enumerated afterwards (camera initial_reset, replugged cables)
# are otherwise invisible inside it.
end=$(( $(date +%s) + ${1:-240} ))
while [ $(date +%s) -lt $end ]; do
  for f in /dev/bus/usb/*/* /dev/ttyUSB* /dev/video* /dev/media*; do
    [ -c "$f" ] || continue
    maj=$(printf %d 0x$(stat -c %t "$f")); min=$(printf %d 0x$(stat -c %T "$f"))
    docker exec -u root isaac_ros_realsense sh -c \
      "if [ ! -e $f ] || [ \"\$(stat -c '%t:%T' $f)\" != \"$(stat -c '%t:%T' "$f")\" ]; then rm -f $f; mkdir -p \$(dirname $f); mknod -m 666 $f c $maj $min; echo synced $f; fi" 2>/dev/null
  done
  sleep 3
done
