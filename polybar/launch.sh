#!/usr/bin/env bash
set -euo pipefail
pkill -x polybar || true
# Optional: export battery/AC names if the defaults don’t work for your board.
# export BATT=axp20x-battery
# export ACAD=axp20x-ac
polybar -r main --config="$HOME/.config/polybar/config.ini" &
disown
