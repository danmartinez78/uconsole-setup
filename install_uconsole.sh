#!/usr/bin/env bash
#
# install_uconsole.sh — Setup script for Dan's uConsole (Ubuntu 24.04, aarch64)
#
# What this does (idempotent, with backups):
#   • Installs i3, polybar, kitty, rofi, picom, feh, scrot, sensors, audio tools
#   • Installs Nerd Fonts (JetBrainsMono Nerd Font) system‑wide if missing
#   • Drops a clean Polybar config + launch script (with icons, wifi, IP, CPU, MEM, Temp, Disk, Volume, Battery, Clock)
#   • Adds host-side Docker/ROS2 aliases under ~/.bashrc.d/
#   • Creates a docker-compose.ros2.yml for ros:humble with host networking
#   • Creates persistent in‑container ROS2 aliases via /etc/profile.d mount
#   • Leaves existing files in place (backs up with timestamp before overwriting)
#
# Safe to re-run. Requires sudo.
#
set -euo pipefail

### Helpers
STAMP() { date +%Y%m%d-%H%M%S; }
BK() { local p="$1"; [[ -e "$p" ]] && cp -a "$p"{".bak.$(STAMP)"} && echo "[backup] $p -> $p.bak.$(STAMP)" || true; }
ENSURE_DIR() { mkdir -p "$1"; }
HAS() { command -v "$1" >/dev/null 2>&1; }

### Guardrails
if [[ $(id -u) -eq 0 ]]; then
  echo "Do not run as root. This script will sudo as needed." >&2
  exit 1
fi

if [[ ! -f /etc/os-release ]] || ! grep -q 'Ubuntu' /etc/os-release; then
  echo "This script targets Ubuntu. Aborting." >&2
  exit 1
fi

echo "==> Updating apt index"
sudo apt-get update -y

### Packages
PKGS=( \
  i3 i3status feh picom rofi kitty polybar \
  fonts-noto-color-emoji fonts-noto fonts-font-awesome \
  scrot brightnessctl pulseaudio pavucontrol \
  lm-sensors hddtemp jq curl git unzip \
  ca-certificates gnupg \
)

# Docker engine (if not present)
if ! HAS docker; then
  echo "==> Installing Docker Engine"
  sudo install -m 0755 -d /etc/apt/keyrings
  curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
  sudo chmod a+r /etc/apt/keyrings/docker.gpg
  . /etc/os-release
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu $VERSION_CODENAME stable" | \
    sudo tee /etc/apt/sources.list.d/docker.list >/dev/null
  sudo apt-get update -y
  PKGS+=( docker-ce docker-ce-cli containerd.io docker-buildx-plugin )
fi

# docker-compose (classic) for this device
if ! HAS docker-compose; then
  PKGS+=( docker-compose )
fi

echo "==> Installing packages: ${PKGS[*]}"
sudo apt-get install -y "${PKGS[@]}"

# Ensure user is in docker group
if ! id -nG "$USER" | grep -qw docker; then
  echo "==> Adding $USER to docker group (you must log out/in or reboot afterward)"
  sudo usermod -aG docker "$USER"
fi

### Nerd Font: JetBrainsMono (system‑wide, if missing)
NErd_TTF="/usr/local/share/fonts/JetBrainsMonoNerdFont-Regular.ttf"
if [[ ! -f "$NErd_TTF" ]]; then
  echo "==> Installing JetBrainsMono Nerd Font"
  TMPF=$(mktemp -d)
  pushd "$TMPF" >/dev/null
  # Minimal single-file download (regular)
  curl -fL -o JetBrainsMonoNerdFont-Regular.ttf \
    https://github.com/ryanoasis/nerd-fonts/raw/refs/heads/master/patched-fonts/JetBrainsMono/Regular/JetBrainsMonoNerdFont-Regular.ttf
  sudo mkdir -p /usr/local/share/fonts
  sudo install -m 0644 JetBrainsMonoNerdFont-Regular.ttf /usr/local/share/fonts/
  sudo fc-cache -f
  popd >/dev/null
  rm -rf "$TMPF"
fi

### Polybar configuration
POLY_DIR="$HOME/.config/polybar"
ENSURE_DIR "$POLY_DIR"

# Backup previous config if present
BK "$POLY_DIR/config.ini"

cat > "$POLY_DIR/config.ini" <<'CFG'
[bar/main]
width = 100%
height = 26
fixed-center = true
padding = 2
line-size = 2
background = #CC111111
foreground = #EEEEEE

# Fonts: JetBrains (text/icons), Noto Emoji for fallback
font-0 = JetBrainsMono Nerd Font:style=Regular:size=12;2
font-1 = Noto Color Emoji:scale=0.9

modules-left  = xwindow sep cpu memory temperature filesystem
modules-right = pulseaudio wlan ip battery date

tray-position = right
tray-padding = 2

[module/xwindow]
type = internal/xwindow
label = %title%
label-maxlen = 40

[module/sep]
type = custom/text
format =  |
format-foreground = #555555

[module/cpu]
type = internal/cpu
interval = 2
format = <label>
label =  %percentage%%

[module/memory]
type = internal/memory
interval = 5
format = <label>
label =  %used% / %total%

[module/temperature]
type = internal/temperature
interval = 5
# Try common sensors; adjust if needed
thermal-zone = 0
hwmon-path = /sys/class/thermal/thermal_zone0/temp
format = <label>
label =  %temperature-c%

[module/filesystem]
type = internal/fs
interval = 30
mount-0 = /
label-mounted =  %free%
label-unmounted =  n/a

[module/pulseaudio]
type = internal/pulseaudio
use-ui-max = true
interval = 2
format-volume =   %percentage%%
format-muted  =   muted

[module/wlan]
type = internal/network
interface = wlan0
interval = 5
format-connected =   %essid% %signal%%
format-disconnected =   offline

[module/ip]
type = custom/script
interval = 5
exec = sh -c "ip -4 addr show wlan0 | awk '/inet /{print \$2}' | cut -d/ -f1 | head -n1 || echo -"
label =  %output%

[module/battery]
type = internal/battery
battery = BAT0
adapter = AC
full-at = 98
poll-interval = 10
format-charging =   %percentage%%
format-discharging = 🔋 %percentage%%
format-full = 🔋 100%

[module/date]
type = internal/date
interval = 1
label =  %Y-%m-%d  %H:%M
CFG

# Launch script
BK "$POLY_DIR/launch.sh"
cat > "$POLY_DIR/launch.sh" <<'LAUNCH'
#!/usr/bin/env bash
set -euo pipefail
pkill -x polybar || true
# Give i3/X time to settle when called from autostart
sleep 0.3
polybar -r main --config="$HOME/.config/polybar/config.ini" &
LAUNCH
chmod +x "$POLY_DIR/launch.sh"

# Ensure i3 autostarts polybar (if not already present)
I3CFG="$HOME/.config/i3/config"
if [[ -f "$I3CFG" ]] && ! grep -q 'polybar/launch.sh' "$I3CFG"; then
  echo "==> Adding polybar autostart to i3 config"
  BK "$I3CFG"
  printf '\n# -- Polybar autostart --\nexec_always --no-startup-id "$HOME/.config/polybar/launch.sh"\n' >> "$I3CFG"
fi

### Host shell aliases for Docker/ROS2
BRC_DIR="$HOME/.bashrc.d"
ENSURE_DIR "$BRC_DIR"
ALIAS_FILE="$BRC_DIR/ros2_docker_aliases.bash"
BK "$ALIAS_FILE"
cat > "$ALIAS_FILE" <<'ALIAS'
# ROS2 / Docker helpers (host)
alias dr-up='docker-compose -f "$HOME"/docker-compose.ros2.yml up -d'
alias dr-down='docker-compose -f "$HOME"/docker-compose.ros2.yml down'
alias dr-bash='docker exec -it ros2 bash'
alias dr-logs='docker logs -f ros2'
ALIAS

# Source ~/.bashrc.d/* from ~/.bashrc if not already
if ! grep -q "\.bashrc\.d" "$HOME/.bashrc"; then
  cat >> "$HOME/.bashrc" <<'SRC'
# Load per-feature aliases & extras
if [[ -d "$HOME/.bashrc.d" ]]; then
  for f in "$HOME"/.bashrc.d/*.bash; do [ -r "$f" ] && source "$f"; done
fi
SRC
fi

### Docker Compose for ROS2 Humble
COMPOSE="$HOME/docker-compose.ros2.yml"
BK "$COMPOSE"
cat > "$COMPOSE" <<'YAML'
version: "3.8"
services:
  ros2:
    image: ros:humble
    container_name: ros2
    network_mode: "host"
    environment:
      - ROS_DOMAIN_ID=0
    volumes:
      - ${HOME}/ros2_ws:/ros2_ws
      - ${HOME}/ros2_ws/config/container_aliases.bash:/etc/profile.d/ros2-aliases.sh:ro
    working_dir: /ros2_ws
    tty: true
    stdin_open: true
YAML

### In‑container persistent aliases
ENSURE_DIR "$HOME/ros2_ws/config"
if [[ ! -f "$HOME/ros2_ws/config/container_aliases.bash" ]]; then
  cat > "$HOME/ros2_ws/config/container_aliases.bash" <<'INC'
# Persistent ROS2 in‑container aliases
# Auto-sourced via /etc/profile.d/ros2-aliases.sh
if [ -f /opt/ros/humble/setup.bash ]; then
  . /opt/ros/humble/setup.bash
fi
alias rlist='ros2 topic list'
alias rhz='ros2 topic hz'      # usage: rhz /topic
alias recho='ros2 topic echo'  # usage: recho /topic
alias rnodes='ros2 node list'
alias rinfo='echo Nodes:; ros2 node list; echo; echo Topics:; ros2 topic list'
rgrep(){ ros2 topic list | grep -i "$1" ; }
export -f rgrep 2>/dev/null || true
INC
fi

### Sensors setup (non-interactive)
# You can run `sudo sensors-detect` manually later if needed.

### Final tips
cat <<'TIP'

==> Install complete.
• Reboot (or log out/in) so: docker group membership, fonts, and i3 autostarts take effect.
• Start your ROS stack:
    dr-up      # compose up (host network)
    dr-bash    # hop into the container shell (ROS auto-sourced)
• Launch Polybar right now (from a local i3 terminal, not SSH):
    ~/.config/polybar/launch.sh
• If any icons appear as squares, confirm fonts are installed and run:  fc-cache -f
• Edit Polybar modules here:  ~/.config/polybar/config.ini

TIP
