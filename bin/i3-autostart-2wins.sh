#!/usr/bin/env bash
set -euo pipefail

FLAG="/run/user/$(id -u)/i3.autostart_2wins.ran"
[ -e "$FLAG" ] && exit 0

# Wait for X session to be ready
for i in $(seq 1 10); do
  if xprop -root >/dev/null 2>&1; then break; fi
  sleep 0.5
done

# Move to workspace 1
i3-msg 'workspace 1:' >/dev/null

# Launch cool-login asynchronously
if command -v cool-login >/dev/null 2>&1; then
  (kitty --class coollogin -e bash -lc "cool-login || neofetch; exec bash") &
else
  (kitty --class coollogin -e bash -lc "neofetch || echo 'Welcome'; exec bash") &
fi

# Give the terminal a moment to spawn, then split horizontally and open pipes
sleep 1
i3-msg 'split h' >/dev/null

if command -v pipes.sh >/dev/null 2>&1; then
  (kitty --class pipes -e bash -lc "pipes.sh -p 2 -t 0; exec bash") &
else
  (kitty --class pipes -e bash -lc "echo pipes.sh not installed; read -n1 -rsp \"press any key\"; exec bash") &
fi

# Mark this session complete
: > "$FLAG"
