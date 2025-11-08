#!/usr/bin/env bash
STATE="$HOME/.cache/i3-split.state"

# Seed state by screen shape on first run
if [[ ! -f "$STATE" ]]; then
  read -r W H < <(i3-msg -t get_outputs | jq -r '.[] | select(.active==true) | "\(.rect.width) \(.rect.height)"' | head -n1)
  if [[ -z "$W" || -z "$H" ]]; then W=1920; H=1080; fi
  # If landscape, start with vertical split (side-by-side); if portrait, start horizontal
  echo $([[ $W -gt $H ]] && echo "v" || echo "h") > "$STATE"
fi

mode=$(cat "$STATE" 2>/dev/null || echo v)

# Decide split and flip for next time (simple, reliable)
if [[ "$mode" == "v" ]]; then
  i3-msg split v >/dev/null
  echo "h" > "$STATE"
else
  i3-msg split h >/dev/null
  echo "v" > "$STATE"
fi

# Launch terminal
exec kitty &
