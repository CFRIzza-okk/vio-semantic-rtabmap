#!/bin/bash
# Runs the rtabmap_core build with -j1 and logs memory every 30s.
WS=/home/jetson/VIO/rtabmap_cuda_ws
cd $WS/build
MON=$WS/logs/memory.log
echo "# t_epoch avail_MB used_MB swap_used_MB max_proc_rss_MB" > $MON
( while true; do
    read -r _ tot used free _ _ avail <<< "$(free -m | sed -n 2p)"
    sw=$(free -m | awk '/Swap|交换/{print $3}')
    maxrss=$(ps -eo rss --no-headers | sort -rn | head -1)
    printf "%s %s %s %s %s\n" "$(date +%s)" "$avail" "$used" "$sw" "$((maxrss/1024))" >> $MON
    sleep 30
  done ) &
MONPID=$!
START=$(date +%s)
echo "BUILD_START $(date -Is) jobs=1" > $WS/logs/build.log
make -j1 rtabmap_core >> $WS/logs/build.log 2>&1
RC=$?
END=$(date +%s)
kill $MONPID 2>/dev/null
echo "BUILD_END $(date -Is) rc=$RC elapsed=$((END-START))s" >> $WS/logs/build.log
awk 'NR>1{if($2<min||min==""){min=$2}; if($5>pk)pk=$5} END{printf "MEM_SUMMARY min_avail_MB=%s peak_max_proc_rss_MB=%s\n",min,pk}' $MON >> $WS/logs/build.log
echo "RC=$RC elapsed=$((END-START))s"
