#!/usr/bin/env bash
#
# cm5-optimize.sh — CM5-specific system optimizations for uConsole
#
# Applies safe, conservative tuning for:
#   • Memory management (swappiness, cache pressure)
#   • zram setup (compressed swap in RAM)
#   • eMMC longevity (noatime, tmpfs)
#   • CPU governor (schedutil for better responsiveness)
#   • Optional: disable unused services
#
# Safe to run multiple times (idempotent).
# Requires sudo for system changes.
#
set -euo pipefail

### Guardrails
if [[ $(id -u) -eq 0 ]]; then
  echo "❌ Do not run as root. This script will sudo as needed." >&2
  exit 1
fi

echo "╔═══════════════════════════════════════════════════════════════╗"
echo "║        CM5 Optimization Script for uConsole                   ║"
echo "╚═══════════════════════════════════════════════════════════════╝"
echo ""
echo "This will apply conservative optimizations for:"
echo "  • 16GB RAM tuning (low swappiness, zram)"
echo "  • eMMC longevity (reduced writes)"
echo "  • Better CPU responsiveness (schedutil governor)"
echo "  • Optional service cleanup"
echo ""
read -p "Continue? (y/N) " -n 1 -r
echo
[[ ! $REPLY =~ ^[Yy]$ ]] && exit 0

### Memory & Swap Tuning
echo "==> Configuring memory management"

# Lower swappiness (default 60 → 10 for systems with plenty of RAM)
if ! grep -q "vm.swappiness" /etc/sysctl.conf 2>/dev/null; then
  echo "vm.swappiness = 10" | sudo tee -a /etc/sysctl.conf >/dev/null
  echo "[✓] Set vm.swappiness = 10"
else
  echo "[✓] vm.swappiness already configured"
fi

# Reduce cache pressure (better for filesystem caching)
if ! grep -q "vm.vfs_cache_pressure" /etc/sysctl.conf 2>/dev/null; then
  echo "vm.vfs_cache_pressure = 50" | sudo tee -a /etc/sysctl.conf >/dev/null
  echo "[✓] Set vm.vfs_cache_pressure = 50"
else
  echo "[✓] vm.vfs_cache_pressure already configured"
fi

# Apply sysctl changes
sudo sysctl -p >/dev/null 2>&1
echo "[✓] Memory tuning applied"

### zram Setup
echo "==> Configuring zram (compressed RAM swap)"

if ! lsmod | grep -q zram; then
  # Install zram-config package if not present
  if ! dpkg -l | grep -q zram-config; then
    echo "Installing zram-config..."
    sudo apt-get update -qq
    sudo apt-get install -y zram-config
  fi
  sudo modprobe zram
  echo "[✓] zram module loaded"
else
  echo "[✓] zram already active"
fi

# Configure zram size (4GB compressed swap in RAM)
if [[ ! -f /etc/default/zramswap ]]; then
  sudo tee /etc/default/zramswap >/dev/null <<'ZRAM'
# zram configuration for CM5 with 16GB RAM
ALGO=lz4
PERCENT=25
PRIORITY=100
ZRAM

  sudo systemctl restart zramswap.service 2>/dev/null || true
  echo "[✓] zram configured (4GB compressed swap)"
else
  echo "[✓] zram already configured"
fi

### eMMC Optimizations
echo "==> Configuring eMMC optimizations"

# Add noatime to root mount if not present
if ! grep -q "noatime" /etc/fstab 2>/dev/null; then
  echo ""
  echo "⚠️  To reduce eMMC writes, add 'noatime,nodiratime' to your root mount in /etc/fstab"
  echo "Example: /dev/mmcblk0p2 / ext4 defaults,noatime,nodiratime 0 1"
  echo ""
  read -p "Open /etc/fstab for editing now? (y/N) " -n 1 -r
  echo
  if [[ $REPLY =~ ^[Yy]$ ]]; then
    sudo nano /etc/fstab
    echo "[!] Changes will take effect after reboot"
  fi
else
  echo "[✓] noatime already configured in fstab"
fi

# tmpfs for /tmp (reduce eMMC writes)
if ! grep -q "tmpfs /tmp" /etc/fstab 2>/dev/null; then
  echo "tmpfs /tmp tmpfs defaults,noatime,mode=1777,size=2G 0 0" | sudo tee -a /etc/fstab >/dev/null
  echo "[✓] tmpfs configured for /tmp (takes effect after reboot)"
else
  echo "[✓] tmpfs already configured for /tmp"
fi

# Enable fstrim timer for eMMC TRIM
if ! systemctl is-enabled fstrim.timer >/dev/null 2>&1; then
  sudo systemctl enable fstrim.timer
  sudo systemctl start fstrim.timer
  echo "[✓] fstrim.timer enabled (weekly TRIM)"
else
  echo "[✓] fstrim.timer already enabled"
fi

### CPU Governor
echo "==> Configuring CPU governor"

# Check current governor
CURRENT_GOV=$(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor 2>/dev/null || echo "unknown")
echo "Current governor: $CURRENT_GOV"

# Set to schedutil (best balance of performance and power)
if [[ "$CURRENT_GOV" != "schedutil" ]]; then
  echo "schedutil" | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor >/dev/null 2>&1 || true
  
  # Make persistent via /etc/rc.local or systemd service
  if [[ ! -f /etc/systemd/system/cpu-governor.service ]]; then
    sudo tee /etc/systemd/system/cpu-governor.service >/dev/null <<'CPUGOV'
[Unit]
Description=Set CPU governor to schedutil
After=multi-user.target

[Service]
Type=oneshot
ExecStart=/bin/bash -c 'echo schedutil | tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor'
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
CPUGOV

    sudo systemctl daemon-reload
    sudo systemctl enable cpu-governor.service
    echo "[✓] CPU governor set to schedutil (persistent)"
  else
    echo "[✓] CPU governor service already exists"
  fi
else
  echo "[✓] Already using schedutil governor"
fi

### Optional: Service Cleanup
echo ""
echo "==> Optional: Disable unused services"
echo "This can save RAM and speed up boot time."
echo ""

# Bluetooth
if systemctl is-enabled bluetooth.service >/dev/null 2>&1; then
  read -p "Disable Bluetooth? (saves ~20MB RAM, faster boot) (y/N) " -n 1 -r
  echo
  if [[ $REPLY =~ ^[Yy]$ ]]; then
    sudo systemctl disable bluetooth.service
    sudo systemctl stop bluetooth.service 2>/dev/null || true
    echo "[✓] Bluetooth disabled"
  fi
fi

# CUPS (printing)
if systemctl is-enabled cups.service >/dev/null 2>&1; then
  read -p "Disable CUPS printing service? (unlikely needed on uConsole) (y/N) " -n 1 -r
  echo
  if [[ $REPLY =~ ^[Yy]$ ]]; then
    sudo systemctl disable cups.service
    sudo systemctl stop cups.service 2>/dev/null || true
    echo "[✓] CUPS disabled"
  fi
fi

# ModemManager
if systemctl is-enabled ModemManager.service >/dev/null 2>&1; then
  read -p "Disable ModemManager? (only needed for cellular modems) (y/N) " -n 1 -r
  echo
  if [[ $REPLY =~ ^[Yy]$ ]]; then
    sudo systemctl disable ModemManager.service
    sudo systemctl stop ModemManager.service 2>/dev/null || true
    echo "[✓] ModemManager disabled"
  fi
fi

### Summary
cat <<'EOF'

╔═══════════════════════════════════════════════════════════════╗
║              ✅ CM5 Optimization Complete!                    ║
╚═══════════════════════════════════════════════════════════════╝

Applied optimizations:
  ✓ Memory tuning (swappiness=10, cache_pressure=50)
  ✓ zram enabled (4GB compressed swap in RAM)
  ✓ eMMC optimizations (fstrim, tmpfs for /tmp)
  ✓ CPU governor set to schedutil
  ✓ Optional services disabled (if selected)

Next steps:
  1. Reboot for all changes to take effect:
     sudo reboot

  2. Verify optimizations after reboot:
     • Check governor:    cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor
     • Check swappiness:  cat /proc/sys/vm/swappiness
     • Check zram:        zramctl
     • Check tmpfs:       df -h | grep tmpfs

  3. Use power-mode script to toggle performance modes:
     power-mode performance   # Max performance
     power-mode balanced      # Default (schedutil)
     power-mode powersave     # Battery saver

  4. Monitor temps in Polybar (throttle warning shows if >75°C)

Enjoy your optimized CM5! 🚀

EOF
