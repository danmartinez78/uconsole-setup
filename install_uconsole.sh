#!/usr/bin/env bash
#
# install_uconsole.sh — uConsole i3 Environment Setup
#
# Installs and configures a complete i3 desktop environment with:
#   • i3wm, Polybar (retro terminal theme), Kitty, Rofi, Picom
#   • JetBrainsMono Nerd Font
#   • Colored shell prompt & dircolors
#   • Docker + ROS2 Humble setup
#   • Utility scripts for autotiling, layouts, etc.
#
# Safe to re-run (backs up existing files with timestamps).
# Requires sudo for package installation only.
#
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

### Helpers
STAMP() { date +%Y%m%d-%H%M%S; }
BK() { 
  local p="$1"
  if [[ -e "$p" ]]; then
    local bk="${p}.bak.$(STAMP)"
    cp -a "$p" "$bk"
    echo "[✓] Backed up: $p -> $(basename "$bk")"
  fi
}
ENSURE_DIR() { mkdir -p "$1"; }
HAS() { command -v "$1" >/dev/null 2>&1; }

### Guardrails
if [[ $(id -u) -eq 0 ]]; then
  echo "❌ Do not run as root. This script will sudo as needed." >&2
  exit 1
fi

if [[ ! -f /etc/os-release ]] || ! grep -qiE '(ubuntu|debian)' /etc/os-release; then
  echo "⚠️  This script targets Ubuntu/Debian."
  read -p "Continue anyway? (y/N) " -n 1 -r
  echo
  [[ ! $REPLY =~ ^[Yy]$ ]] && exit 1
fi

echo "╔═══════════════════════════════════════════════════════════════╗"
echo "║          uConsole i3 Environment Installer                    ║"
echo "╚═══════════════════════════════════════════════════════════════╝"
echo ""

### Update package index
echo "==> Updating apt index"
sudo apt-get update -y

### Install packages
PKGS=( \
  i3 i3status feh picom rofi kitty polybar \
  fonts-noto-color-emoji fonts-noto fonts-font-awesome \
  scrot brightnessctl pulseaudio pavucontrol \
  lm-sensors hddtemp jq curl git unzip \
  python3-i3ipc \
  ca-certificates gnupg \
)

# Docker engine (if not present)
if ! HAS docker; then
  echo "==> Installing Docker Engine"
  sudo install -m 0755 -d /etc/apt/keyrings
  curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg 2>/dev/null || true
  sudo chmod a+r /etc/apt/keyrings/docker.gpg
  . /etc/os-release
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu ${VERSION_CODENAME:-jammy} stable" | \
    sudo tee /etc/apt/sources.list.d/docker.list >/dev/null
  sudo apt-get update -y
  PKGS+=( docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin )
fi

echo "==> Installing packages"
sudo apt-get install -y "${PKGS[@]}"

# Add user to docker group
if ! id -nG "$USER" | grep -qw docker; then
  echo "==> Adding $USER to docker group"
  sudo usermod -aG docker "$USER"
  echo "[!] You must log out and log back in for docker group to take effect"
fi

### JetBrainsMono Nerd Font (user-local)
FONT_DIR="$HOME/.local/share/fonts"
ENSURE_DIR "$FONT_DIR"
if [[ ! -f "$FONT_DIR/JetBrainsMonoNerdFont-Regular.ttf" ]]; then
  echo "==> Installing JetBrainsMono Nerd Font"
  TMPF=$(mktemp -d)
  pushd "$TMPF" >/dev/null
  curl -fsSL -o JetBrainsMonoNerdFont-Regular.ttf \
    "https://github.com/ryanoasis/nerd-fonts/raw/refs/heads/master/patched-fonts/JetBrainsMono/Ligatures/Regular/JetBrainsMonoNerdFont-Regular.ttf"
  curl -fsSL -o JetBrainsMonoNerdFont-Bold.ttf \
    "https://github.com/ryanoasis/nerd-fonts/raw/refs/heads/master/patched-fonts/JetBrainsMono/Ligatures/Bold/JetBrainsMonoNerdFont-Bold.ttf"
  install -m 0644 JetBrainsMono*.ttf "$FONT_DIR/"
  fc-cache -f
  popd >/dev/null
  rm -rf "$TMPF"
  echo "[✓] JetBrainsMono Nerd Font installed"
fi

### i3 configuration
I3_DIR="$HOME/.config/i3"
ENSURE_DIR "$I3_DIR"
if [[ -f "$REPO_DIR/i3/config" ]]; then
  BK "$I3_DIR/config"
  cp "$REPO_DIR/i3/config" "$I3_DIR/config"
  echo "[✓] i3 config installed"
fi

if [[ -d "$REPO_DIR/i3/layouts" ]]; then
  ENSURE_DIR "$I3_DIR/layouts"
  cp "$REPO_DIR/i3/layouts"/*.json "$I3_DIR/layouts/" 2>/dev/null || true
  echo "[✓] i3 layouts installed"
fi

### Kitty configuration
KITTY_DIR="$HOME/.config/kitty"
ENSURE_DIR "$KITTY_DIR"
if [[ -f "$REPO_DIR/kitty/kitty.conf" ]]; then
  BK "$KITTY_DIR/kitty.conf"
  cp "$REPO_DIR/kitty/kitty.conf" "$KITTY_DIR/kitty.conf"
  echo "[✓] Kitty config installed"
fi

### Rofi theme
ROFI_DIR="$HOME/.config/rofi"
ENSURE_DIR "$ROFI_DIR"
if [[ -f "$REPO_DIR/rofi/dracula.rasi" ]]; then
  BK "$ROFI_DIR/dracula.rasi"
  cp "$REPO_DIR/rofi/dracula.rasi" "$ROFI_DIR/dracula.rasi"
  echo "[✓] Rofi theme installed"
fi

### Polybar configuration (retro terminal theme)
POLY_DIR="$HOME/.config/polybar"
ENSURE_DIR "$POLY_DIR"

if [[ -f "$REPO_DIR/polybar/config.ini" ]]; then
  BK "$POLY_DIR/config.ini"
  cp "$REPO_DIR/polybar/config.ini" "$POLY_DIR/config.ini"
  echo "[✓] Polybar config installed (retro terminal theme)"
fi

if [[ -f "$REPO_DIR/polybar/launch.sh" ]]; then
  BK "$POLY_DIR/launch.sh"
  cp "$REPO_DIR/polybar/launch.sh" "$POLY_DIR/launch.sh"
  chmod +x "$POLY_DIR/launch.sh"
  echo "[✓] Polybar launch script installed"
fi

### i3status configuration
I3STATUS_DIR="$HOME/.config/i3status"
ENSURE_DIR "$I3STATUS_DIR"
if [[ -f "$REPO_DIR/i3status/config" ]]; then
  BK "$I3STATUS_DIR/config"
  cp "$REPO_DIR/i3status/config" "$I3STATUS_DIR/config"
  echo "[✓] i3status config installed"
fi

### Utility scripts
BIN_DIR="$HOME/.local/bin"
ENSURE_DIR "$BIN_DIR"

if [[ -d "$REPO_DIR/bin" ]]; then
  for script in "$REPO_DIR/bin"/*; do
    [[ -f "$script" ]] || continue
    name=$(basename "$script")
    BK "$BIN_DIR/$name"
    cp "$script" "$BIN_DIR/$name"
    chmod +x "$BIN_DIR/$name"
  done
  echo "[✓] Utility scripts installed to ~/.local/bin"
fi

# Ensure ~/.local/bin is in PATH
if ! echo "$PATH" | grep -q "$HOME/.local/bin"; then
  if ! grep -q '.local/bin' "$HOME/.bashrc" 2>/dev/null; then
    echo 'export PATH="$HOME/.local/bin:$PATH"' >> "$HOME/.bashrc"
    echo "[✓] Added ~/.local/bin to PATH"
  fi
fi

### Shell colors (prompt user)
echo ""
read -p "Install colored shell prompt and dircolors? (Y/n) " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Nn]$ ]]; then
  BRC_DIR="$HOME/.bashrc.d"
  ENSURE_DIR "$BRC_DIR"
  
  if [[ -f "$REPO_DIR/bashrc.d/10-color-prompt.bash" ]]; then
    BK "$BRC_DIR/10-color-prompt.bash"
    cp "$REPO_DIR/bashrc.d/10-color-prompt.bash" "$BRC_DIR/10-color-prompt.bash"
  fi
  
  if [[ -f "$REPO_DIR/bashrc.d/20-dircolors-dracula.bash" ]]; then
    BK "$BRC_DIR/20-dircolors-dracula.bash"
    cp "$REPO_DIR/bashrc.d/20-dircolors-dracula.bash" "$BRC_DIR/20-dircolors-dracula.bash"
  fi
  
  # Ensure ~/.bashrc.d sourcing
  if ! grep -q "\.bashrc\.d" "$HOME/.bashrc" 2>/dev/null; then
    cat >> "$HOME/.bashrc" <<'SRC'

# Load per-feature aliases & extras
if [[ -d "$HOME/.bashrc.d" ]]; then
  for f in "$HOME"/.bashrc.d/*.bash; do [ -r "$f" ] && source "$f"; done
fi
SRC
  fi
  echo "[✓] Shell colors installed"
fi

### Docker/ROS2 aliases
BRC_DIR="$HOME/.bashrc.d"
ENSURE_DIR "$BRC_DIR"
ALIAS_FILE="$BRC_DIR/ros2_docker_aliases.bash"
BK "$ALIAS_FILE"
cat > "$ALIAS_FILE" <<'ALIAS'
# ROS2 / Docker helpers (host)
alias dr-up='docker compose -f "$HOME/docker-compose.ros2.yml" up -d'
alias dr-down='docker compose -f "$HOME/docker-compose.ros2.yml" down'
alias dr-bash='docker exec -it ros2 bash'
alias dr-logs='docker logs -f ros2'
ALIAS
echo "[✓] Docker/ROS2 aliases installed"

# Ensure ~/.bashrc.d sourcing
if ! grep -q "\.bashrc\.d" "$HOME/.bashrc" 2>/dev/null; then
  cat >> "$HOME/.bashrc" <<'SRC'

# Load per-feature aliases & extras
if [[ -d "$HOME/.bashrc.d" ]]; then
  for f in "$HOME"/.bashrc.d/*.bash; do [ -r "$f" ] && source "$f"; done
fi
SRC
fi

### Docker Compose for ROS2
if [[ -f "$REPO_DIR/docker-compose.ros2.yml" ]]; then
  BK "$HOME/docker-compose.ros2.yml"
  cp "$REPO_DIR/docker-compose.ros2.yml" "$HOME/docker-compose.ros2.yml"
  echo "[✓] Docker Compose for ROS2 installed"
fi

### ROS2 workspace
ROS_WS="$HOME/ros2_ws"
ENSURE_DIR "$ROS_WS/config"

if [[ -f "$REPO_DIR/ros2_ws/config/container_aliases.bash" ]]; then
  BK "$ROS_WS/config/container_aliases.bash"
  cp "$REPO_DIR/ros2_ws/config/container_aliases.bash" "$ROS_WS/config/container_aliases.bash"
  echo "[✓] ROS2 container aliases installed"
fi

### Wallpapers
if [[ -d "$REPO_DIR/wallpapers" ]]; then
  WALL_DIR="$HOME/.local/share/wallpapers"
  ENSURE_DIR "$WALL_DIR"
  cp "$REPO_DIR/wallpapers"/* "$WALL_DIR/" 2>/dev/null || true
  echo "[✓] Wallpapers installed"
fi

### Final message
cat <<'EOF'

╔═══════════════════════════════════════════════════════════════════╗
║                  ✅ Installation Complete!                        ║
╚═══════════════════════════════════════════════════════════════════╝

Next steps:
  1. Log out and log back in (or reboot) for changes to take effect
  
  2. Start i3 from your login screen

  3. Test Polybar:
     ~/.config/polybar/launch.sh

  4. Docker/ROS2 usage:
     dr-up       # Start ROS2 container
     dr-bash     # Enter container shell
     dr-down     # Stop container

  5. Optional:
     • Run 'sudo sensors-detect' to configure temp sensors
     • Customize ~/.config/polybar/config.ini
     • Edit workspace names in ~/.config/i3/config

Enjoy your retro terminal uConsole! 🟢

To uninstall, run: ./uninstall_uconsole.sh

EOF
