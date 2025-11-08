# 🧠 uConsole Setup Environment

This repository contains a fully configured **uConsole developer environment** for ROS 2, Docker, and i3 with Polybar integration.
It’s designed for field-ready robotics and embedded development, optimized for the **ClockworkPi uConsole**.

---

## 🧩 Features

### System

* Lightweight **i3 window manager** setup
* **Polybar** with live system stats, icons, and battery readout
* **Persistent aliases** for Docker + ROS2 workflow
* **Kitty terminal** integration
* **Neofetch** startup display
* Ergonomic i3 keybindings for handheld use

### Development Environment

* **ROS 2 Humble** (Docker-based)
* `docker-compose.ros2.yml` with support for:

  * Fast startup (`docker-compose up -d`)
  * Multicast ROS 2 network discovery
  * Persistent workspace mounts (`~/ros2_ws`)
* **Aliases** for ROS2 CLI shortcuts (`rlist`, `rhx`, `recho`, `rnodes`, `rgrep`)

### Polybar

* Active window title
* CPU %, Memory usage, Temperature, Disk usage
* WLAN IP address
* Audio volume level (with mute icon)
* Battery level with icons and charging indicator
* Clock (HH:MM, 24-hour format)

---

## 🧱 Installation

Clone this repo and run the setup script (if provided):

```bash
git clone https://github.com/danmartinez78/uconsole-i3-setup.git
cd uconsole-i3-setup
```

If you’re restoring on a new uConsole:

```bash
bash install_uconsole.sh
```

Otherwise, you can manually sync your configs:

```bash
rsync -a ./config ~/.config/
rsync -a ./bin ~/.local/bin/
rsync -a ./ros2_ws ~/ros2_ws/
cp -f ./docker-compose.ros2.yml ~/
```

Then reload i3:

```
Mod + Shift + R
```

---

## 🛣️ Docker / ROS 2 Workflow

| Action                   | Command                                             |
| ------------------------ | --------------------------------------------------- |
| Start container          | `docker-compose -f ~/docker-compose.ros2.yml up -d` |
| Stop container           | `docker-compose -f ~/docker-compose.ros2.yml down`  |
| Attach interactive shell | `docker exec -it ros2 bash`                         |
| View logs                | `docker logs -f ros2`                               |

### Common ROS2 Aliases (inside container)

| Alias       | Command                               |
| ----------- | ------------------------------------- |
| `rlist`     | `ros2 topic list`                     |
| `rhx`       | `ros2 topic hz <topic>`               |
| `recho`     | `ros2 topic echo <topic>`             |
| `rnodes`    | `ros2 node list`                      |
| `rgrep foo` | Grep for topic names containing “foo” |

Aliases are auto-sourced via `~/.bashrc` inside the container.

---

## 🎹 i3 Keybindings (Ergonomic uConsole Edition)

| Combo                            | Action                                      |
| -------------------------------- | ------------------------------------------- |
| **Alt + Ctrl + H/J/K/L**         | Move window left/down/up/right              |
| **Alt + Ctrl + Shift + H/J/K/L** | Resize window                               |
| **Alt + Ctrl + Return**          | Attach to ROS2 Docker shell                 |
| **Alt + Ctrl + U / X**           | Start / Stop Docker Compose                 |
| **Alt + Ctrl + O**               | Tail ROS2 logs                              |
| **Alt + Ctrl + P**               | Launch startup animation (Neofetch + Pipes) |
| **Alt + Ctrl + S**               | Take screenshot (region select)             |
| **Alt + Q**                      | Close window                                |

---

## 📊 Polybar Overview

| Module     | Info                                |
| ---------- | ----------------------------------- |
| ** CPU**  | CPU load percentage                 |
| ** MEM**  | Memory used                         |
| ** TEMP** | CPU temperature                     |
| ** DISK** | Root partition usage                |
| ** WIFI** | WLAN IP address                     |
| ** VOL**  | Volume level or mute                |
| ** TIME** | System clock                        |
| ** BAT**  | Battery level or charging indicator |

You can relaunch Polybar without rebooting:

```bash
pkill -x polybar || true
DISPLAY=:0 ~/.config/polybar/launch.sh
## 🎨 Shell Colors (Optional)

Want a colorful prompt and vivid `ls` colors?

- Copy the provided bash snippets into your environment:

```bash
mkdir -p ~/.bashrc.d
cp -f ./bashrc.d/10-color-prompt.bash ~/.bashrc.d/
cp -f ./bashrc.d/20-dircolors-dracula.bash ~/.bashrc.d/
```

Log out/in or `source ~/.bashrc`. The prompt shows time, user@host, cwd, git branch (with `*` if dirty), and Python venv. `ls` uses a Dracula‑ish palette.

```

---

## ⚙️ Troubleshooting

### Fonts / Icons Missing

Make sure Nerd Fonts are installed:

```bash
fc-list | grep -i nerd
```

Recommended:

```
JetBrainsMono Nerd Font
Noto Color Emoji
```

Then rebuild font cache:

```bash
fc-cache -fv
```

### Battery Not Showing

List available devices:

```bash
ls -1 /sys/class/power_supply/
```

Update `[module/battery]` section in `~/.config/polybar/config.ini` with:

```ini
battery = axp20x-battery
adapter = axp20x-ac
```

---

## 🧭 Exporting and Rebuilding

To snapshot the current config for transfer or versioning:

```bash
STAMP=$(date +%Y%m%d-%H%M)
tar -czf ~/uconsole-backup-$STAMP.tar.gz \
    ~/.config/i3 ~/.config/polybar ~/.local/bin ~/docker-compose.ros2.yml ~/ros2_ws
```

On a fresh system, extract and restore:

```bash
tar -xzf uconsole-backup-*.tar.gz -C ~/
```

---

## 🧑‍💻 Author

**Dan** – Drone systems & autonomy architecture
💡 Robotics, mining autonomy, ROS 2, and embedded Linux tinkering.

---

## 📜 License

MIT License — use freely, modify boldly, share improvements.
