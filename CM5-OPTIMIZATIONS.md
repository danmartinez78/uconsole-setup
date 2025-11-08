# CM5 Optimizations

This repository includes **CM5-specific optimizations** for the uConsole with 16GB RAM and eMMC storage.

## 🚀 Quick Start

After running `./install_uconsole.sh`, apply CM5 optimizations:

```bash
cd ~/.local/bin
chmod +x cm5-optimize.sh power-mode check-throttle
./cm5-optimize.sh
```

Then **reboot** to activate all changes.

## ⚡ What Gets Optimized

### Memory Management
- **Swappiness**: Reduced from 60 → 10 (use RAM more, swap less)
- **Cache Pressure**: Tuned to 50 for better filesystem caching
- **zram**: 4GB compressed swap in RAM (no eMMC wear)

### eMMC Longevity
- **noatime/nodiratime**: Reduce write operations
- **tmpfs for /tmp**: RAM-based temp files (2GB)
- **fstrim**: Weekly TRIM for eMMC health

### CPU Performance
- **Governor**: schedutil (balanced performance & efficiency)
- **Toggle modes**: Use `power-mode` to switch between performance/balanced/powersave

### Docker/ROS2
- **Memory limits**: Container limited to 4GB (prevents OOM)
- **Optimized for**: Lightweight telemetry + remote visualization (Foxglove/RViz)

### Polybar
- **Throttle warning**: Shows ⚠️ icon only when CPU >75°C
- **Zero space usage**: Hidden when cool, appears when hot

## 🎛️ Power Mode Toggles

Switch CPU governor on-the-fly:

```bash
power-mode performance   # Max performance (AC power)
power-mode balanced      # Default (schedutil)
power-mode powersave     # Battery saver
power-mode status        # Show current mode & temps
```

## 📊 Monitoring

Check optimizations are active:

```bash
# CPU governor
cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor

# Swappiness
cat /proc/sys/vm/swappiness

# zram status
zramctl

# Memory usage
free -h

# Temperature
cat /sys/class/thermal/thermal_zone0/temp
# (Output is in millidegrees, divide by 1000)
```

## 🔧 Optional Service Cleanup

The `cm5-optimize.sh` script prompts to disable:
- **Bluetooth** (saves ~20MB RAM)
- **CUPS** (printing - unlikely needed)
- **ModemManager** (cellular modems)

You can re-enable any service later:
```bash
sudo systemctl enable bluetooth.service
sudo systemctl start bluetooth.service
```

## 🌡️ Thermal Monitoring

With passive cooling only, expect:
- **Idle**: 45-55°C
- **Light load**: 55-65°C
- **Heavy load**: 65-75°C
- **Throttling**: Starts around 80-85°C

The Polybar throttle indicator will show ⚠️ when >75°C.

## 🔄 Reverting Optimizations

To undo changes:

1. Edit `/etc/sysctl.conf` - remove vm.swappiness and vm.vfs_cache_pressure lines
2. Disable zram: `sudo systemctl disable zramswap.service`
3. Edit `/etc/fstab` - remove noatime and tmpfs entries
4. Disable CPU governor service: `sudo systemctl disable cpu-governor.service`
5. Reboot

## 📝 Notes

- All optimizations are **conservative** and safe
- No GPU/compositor changes (avoids i3/Picom transparency issues)
- No aggressive undervolting (passive cooling only)
- Designed for: 16GB RAM + eMMC + lightweight ROS2 telemetry
