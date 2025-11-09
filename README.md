# 🧠 uConsole i3 Setup

A fully configured **i3 desktop environment** for the **ClockworkPi uConsole CM5**, featuring a retro terminal aesthetic, CM5-specific optimizations, and optional ROS 2 Docker integration for robotics development.

Designed for field-ready development on the handheld uConsole, optimized for both everyday computing and embedded systems work.

---

## 🧩 Features

### Desktop Environment

* Lightweight **i3 window manager** with ergonomic keybindings for handheld use
* **Polybar** status bar with retro terminal theme (green/amber/cyan)
  * Live system stats: CPU, RAM, temperature, disk usage
  * Battery level with color-coded icons
  * WiFi status (SSID, signal, IP), audio volume, uptime
  * Thermal throttle warning (>75°C)
* **Kitty terminal** with JetBrainsMono Nerd Font
* **Colored shell prompt** with git branch indicators and venv support
* **Rofi** application launcher with Dracula theme

### CM5 Optimizations

* **Memory tuning** (swappiness=10, zram compressed swap)
* **eMMC longevity** (noatime, tmpfs, weekly TRIM)
* **CPU governor** (schedutil for balanced performance/battery)
* **Power mode toggle** (performance/balanced/powersave)
* **Thermal monitoring** with real-time Polybar warnings
* See [CM5-OPTIMIZATIONS.md](CM5-OPTIMIZATIONS.md) for full details

### Development Environment (Optional)

* **ROS 2 Humble** Docker-based setup
* `docker-compose.ros2.yml` with:
  * Fast startup (`docker compose up -d`)
  * Multicast ROS 2 network discovery
  * Persistent workspace mounts (`~/ros2_ws`)
* **CLI aliases** for ROS2 workflow (`rlist`, `rhz`, `recho`, `rnodes`, `rgrep`)

---

## 🚀 Installation

Clone this repository:

```bash
git clone https://github.com/danmartinez78/uconsole-i3-setup.git
cd uconsole-i3-setup
```

Run the automated installer:

```bash
./install_uconsole.sh
```

The installer will:
- Install i3, Polybar, Kitty, Rofi, and dependencies
- Set up JetBrainsMono Nerd Font (user-local)
- Copy all configurations to `~/.config/`
- Install utility scripts to `~/.local/bin/`
- **Prompt** for colored shell prompt (optional, default YES)
- **Prompt** for Docker/ROS2 setup (optional, default NO)

Then apply CM5 optimizations (recommended):

```bash
sudo ~/.local/bin/cm5-optimize.sh
```

Log out and select **i3** from your display manager, or reboot.

---

## 🛣️ Docker / ROS 2 Workflow (Optional)

> **Note:** Only available if you enabled Docker/ROS2 during installation.

| Action                   | Alias / Command                                     |
| ------------------------ | --------------------------------------------------- |
| Start container          | `dr-up`                                             |
| Stop container           | `dr-down`                                           |
| Attach interactive shell | `dr-bash`                                           |
| View logs                | `dr-logs`                                           |

### ROS2 Aliases (inside container)

| Alias       | Command                               |
| ----------- | ------------------------------------- |
| `rlist`     | `ros2 topic list`                     |
| `rhz`       | `ros2 topic hz <topic>`               |
| `recho`     | `ros2 topic echo <topic>`             |
| `rnodes`    | `ros2 node list`                      |
| `rgrep foo` | Grep for topic names containing "foo" |

---

## ⚡ CM5 Power Management

Use the `power-mode` utility to toggle CPU performance:

```bash
power-mode performance   # Max CPU speed (AC power)
power-mode balanced      # Default (schedutil, adaptive)
power-mode powersave     # Minimum frequency (battery life)
power-mode status        # Show current governor & temps
```

Monitor optimizations:

```bash
cat /proc/sys/vm/swappiness  # Should show 10
zramctl                      # Should show ~8GB compressed swap
mount | grep noatime         # Root with noatime enabled
```

---

## 🎹 i3 Keybindings (Ergonomic uConsole Edition)

| Combo                            | Action                                      |
| -------------------------------- | ------------------------------------------- |
| **Mod + Enter**                  | Open Kitty terminal                         |
| **Mod + D**                      | Launch Rofi app launcher                    |
| **Mod + Q**                      | Close window                                |
| **Mod + H/J/K/L**                | Focus window left/down/up/right             |
| **Mod + Shift + H/J/K/L**        | Move window                                 |
| **Mod + F**                      | Toggle fullscreen                           |
| **Mod + Shift + Space**          | Toggle floating                             |
| **Mod + 1-9**                    | Switch to workspace 1-9                     |
| **Mod + Shift + 1-9**            | Move window to workspace 1-9                |
| **Mod + Shift + R**              | Reload i3 config                            |
| **Mod + Shift + E**              | Exit i3                                     |

**Custom bindings** (optional, requires ROS2 setup):
- **Mod + Alt + Return**: Attach to ROS2 Docker shell
- **Mod + Alt + U/X**: Start/Stop Docker Compose
- **Mod + Alt + O**: Tail ROS2 logs
- **Mod + Alt + S**: Screenshot (region select)

> Default Mod key: **Alt** (configurable in `~/.config/i3/config`)

---

## 📊 Polybar Overview

| Module       | Info                                |
| ------------ | ----------------------------------- |
| **i3**       | Workspace indicators (1-9)          |
| **󰻠 CPU**    | CPU load percentage                 |
| **󰍛 RAM**    | Memory usage percentage             |
| ** TEMP**   | CPU temperature (amber → red >70°C) |
| **throttle** | Thermal warning (only shows >75°C)  |
| **󰋊 DISK**   | Root partition usage                |
| **󰖩 WIFI**   | SSID, signal %, IP address          |
| **󰕾 VOL**    | Volume level or mute                |
| **󰂎 BAT**    | Battery level with charging icon    |
| **󰔟 Uptime** | System uptime in hours              |
| **󰥔 TIME**   | System clock (HH:MM)                |

Restart Polybar without rebooting:

```bash
pkill -x polybar || true
~/.config/polybar/launch.sh
```

---

## 🎨 Shell Colors

If you enabled colored shell prompt during installation:

**Features:**
- Retro color scheme (amber user, cyan path, green git)
- Git branch indicator with `*` for uncommitted changes
- Python venv display
- Exit code indicator (✘ for failures)

**Dircolors:**
- Green directories
- Amber executables/scripts
- Cyan configs (`.json`, `.yaml`, etc.)
- Red archives

Apply changes:

```bash
source ~/.bashrc
```

---

## ⚙️ Troubleshooting

### Fonts / Icons Missing

Verify Nerd Font installation:

```bash
fc-list | grep -i jetbrains
```

Rebuild font cache if needed:

```bash
fc-cache -fv
```

### Battery Not Showing

Check available power supply devices:

```bash
ls -1 /sys/class/power_supply/
```

For CM5, update `~/.config/polybar/config.ini`:

```ini
[module/battery]
battery = axp20x-battery
adapter = axp22x-ac
```

### Temperature Not Showing

List thermal zones:

```bash
ls /sys/class/thermal/thermal_zone*/type
cat /sys/class/thermal/thermal_zone0/temp
```

For CM5, Polybar uses `thermal_zone0` (cpu-thermal).

---

## 🧭 Backup and Restore

Create a snapshot:

```bash
STAMP=$(date +%Y%m%d-%H%M)
tar -czf ~/uconsole-backup-$STAMP.tar.gz \
    ~/.config/i3 ~/.config/polybar ~/.config/kitty ~/.config/rofi \
    ~/.local/bin ~/docker-compose.ros2.yml ~/ros2_ws ~/.bashrc.d
```

Restore on a fresh system:

```bash
tar -xzf uconsole-backup-*.tar.gz -C ~/
```

---

## 🧑‍💻 Author

**Dan** — Drone systems & autonomy architecture  
💡 Robotics, mining autonomy, ROS 2, and embedded Linux tinkering.

---

## 📜 License

MIT License — use freely, modify boldly, share improvements.
