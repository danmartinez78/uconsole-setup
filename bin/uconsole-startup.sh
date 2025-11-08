#!/usr/bin/env bash
# --- Wait for i3 to start fully ---
sleep 1
i3-msg "workspace 1:"
i3-msg "exec kitty --class coollogin --hold -e bash -lc neofetch"
sleep 1
i3-msg "split h"
i3-msg "exec kitty --class pipes --hold -e bash -lc 'pipes.sh -p 2 -t 2 -R'"
