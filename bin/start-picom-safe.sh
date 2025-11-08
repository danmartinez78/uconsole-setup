#!/usr/bin/env bash
for i in $(seq 1 10); do
    if xprop -root >/dev/null 2>&1; then
        nohup picom --backend xrender --no-fading-openclose >/tmp/picom.autostart.log 2>&1 &
        exit 0
    fi
    sleep 1
done
echo "⚠️ X not ready, compositor skipped" >> /tmp/picom.autostart.log
