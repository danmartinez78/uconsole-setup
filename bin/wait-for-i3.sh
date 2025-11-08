#!/usr/bin/env bash
for i in $(seq 1 20); do
  if i3-msg -t get_workspaces >/dev/null 2>&1; then
    echo "✅ i3 ready after $i tries"
    exit 0
  fi
  sleep 1
done
echo "❌ i3 never became ready" >&2
exit 1
