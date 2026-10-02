#!/bin/bash
# Watchdog: logs memory; if available < 400MB, pauses the build (SIGSTOP on make tree) and notifies.
WS=/home/jetson/VIO/rtabmap_cuda_ws
LOG=$WS/logs/watchdog.log
echo "# watchdog start $(date -Is) threshold=400MB" > $LOG
while true; do
  avail=$(free -m | awk 'NR==2{print $7}')
  echo "$(date +%H:%M:%S) avail=${avail}MB" >> $LOG
  if [ "$avail" -lt 400 ]; then
    echo "$(date -Is) CRITICAL avail=${avail}MB -> SIGSTOP build tree" >> $LOG
    pkill -STOP -f "make -j1 rtabmap_core" 2>/dev/null
    pkill -STOP -f "Makefile2 rtabmap_core" 2>/dev/null
    pkill -STOP -f "cc1plus" 2>/dev/null
    echo "PAUSED_CRITICAL avail=${avail}MB"
    exit 0
  fi
  sleep 20
done
