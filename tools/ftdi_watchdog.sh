#!/bin/bash
# ftdi_watchdog.sh - reset the PX4 serial adapter when the kernel reports it stalled.
#
#   sudo ./tools/ftdi_watchdog.sh            # on the drone host, runs until killed
#
# Field report 4.1: the PX4 adapters are clone FT232RL chips. When one stalls
# ('ftdi_sio ... urb stopped: -32', or its config requests fail with -32), Linux
# stops reading PX4 for good. A USB port reset recovers the adapter, and the
# running MicroXRCEAgent reopens the port on its own, but how fast varied: PX4
# data was back after 2 s in one bench trial and after ~40 s in the next (agent
# reopened at +31 s, then PX4 re-created its session; ghost, 2026-10-28). So a
# reset not followed by PX4 data within a few seconds should be followed by a
# PX4-link (tab 2) restart, which reconnects in ~15 s. That second step is not
# built yet. Needs root for the reset ioctl.
#
# Self-test without a real stall:  echo ftdi_watchdog_selftest | sudo tee /dev/kmsg
VID=0403                                   # FTDI; the LoRa radios are CP2102 (10c4)
USBDEVFS_RESET=21780
STALL='ftdi_sio .*(urb stopped: -32|failed to set baudrate|failed to set flow control: -32|failed to get modem status: -32)|ftdi_watchdog_selftest'

reset_ftdi() {
  for d in /sys/bus/usb/devices/*; do
    [ "$(cat "$d/idVendor" 2>/dev/null)" = "$VID" ] || continue
    dev=$(printf '/dev/bus/usb/%03d/%03d' "$(cat "$d/busnum")" "$(cat "$d/devnum")")
    python3 -c "import fcntl, os; fd = os.open('$dev', os.O_WRONLY); fcntl.ioctl(fd, $USBDEVFS_RESET, 0); os.close(fd)" &&
      echo "$(date +%T) reset $(basename "$d") ($dev)"
  done
}

[ "$(id -u)" = 0 ] || { echo "run as root (the reset ioctl needs it)"; exit 1; }
echo "$(date +%T) watching the kernel log for PX4 adapter stalls"
last=0
dmesg --follow-new | grep --line-buffered -E "$STALL" | while read -r line; do
  echo "$(date +%T) stall: $line"
  now=$(date +%s)
  # one stall logs several lines (two read URBs, then config requests); reset once
  if [ $((now - last)) -ge 5 ]; then
    last=$now
    reset_ftdi
  fi
done
