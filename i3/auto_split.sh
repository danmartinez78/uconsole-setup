#!/usr/bin/env bash
set -euo pipefail
read -r W H < <(i3-msg -t get_tree | jq -r '.. | select(.focused?) | "\(.rect.width) \(.rect.height)"')
if (( W > 2*H )); then
  i3-msg split v >/dev/null
elif (( H > 2*W )); then
  i3-msg split h >/dev/null
else
  i3-msg split toggle >/dev/null
fi
