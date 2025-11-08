#!/usr/bin/env bash
# Outputs: "NN% MM°C" (CPU usage over 1s + CPU temperature)
# Safe, no process substitution; works on busybox/dash/bash
set -euo pipefail

read cpu user nice system idle iowait irq softirq steal guest < /proc/stat
prev_idle=$((idle + iowait))
prev_non_idle=$((user + nice + system + irq + softirq + steal))
prev_total=$((prev_idle + prev_non_idle))

sleep 1

read cpu user nice system idle iowait irq softirq steal guest < /proc/stat
idle2=$((idle + iowait))
non_idle2=$((user + nice + system + irq + softirq + steal))
Total=$((idle2 + non_idle2))

diff_total=$((Total - prev_total))
diff_idle=$((idle2 - prev_idle))

if [ "$diff_total" -le 0 ]; then
  cpu_pct=0
else
  # integer percent with rounding
  cpu_pct=$(( (100 * (diff_total - diff_idle) + diff_total/2) / diff_total ))
fi

# Temperature reading; fallbacks
if [ -f /sys/class/thermal/thermal_zone0/temp ]; then
  t=$(cat /sys/class/thermal/thermal_zone0/temp)
  # Handle values like 42000 or 42
  if [ "$t" -ge 1000 ]; then temp_c=$((t/1000)); else temp_c=$t; fi
else
  temp_c=$(sensors 2>/dev/null | awk '/^Package id 0:|^Tdie:|^temp1:/{print int($2)}; exit')
  temp_c=${temp_c:-0}
fi

echo "${cpu_pct}% ${temp_c}°C"
