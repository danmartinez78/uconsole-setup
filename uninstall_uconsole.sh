#!/usr/bin/env bash
#
# uninstall_uconsole.sh — Remove uConsole i3 Environment
#
# Removes configuration files installed by install_uconsole.sh
# Does NOT uninstall packages (i3, polybar, etc.) - those remain system-wide.
# Backup files (.bak.*) are preserved.
#
set -euo pipefail

echo "╔═══════════════════════════════════════════════════════════════╗"
echo "║          uConsole i3 Environment Uninstaller                  ║"
echo "╚═══════════════════════════════════════════════════════════════╝"
echo ""
echo "This will remove configuration files installed by install_uconsole.sh"
echo "Packages (i3, polybar, kitty, etc.) will remain installed."
echo "Backup files (.bak.*) will be preserved."
echo ""
read -p "Continue? (y/N) " -n 1 -r
echo
[[ ! $REPLY =~ ^[Yy]$ ]] && exit 0

RM() {
  local p="$1"
  if [[ -e "$p" ]]; then
    rm -rf "$p"
    echo "[✓] Removed: $p"
  fi
}

### Remove configs
echo "==> Removing configuration files"

# i3
RM "$HOME/.config/i3/config"
RM "$HOME/.config/i3/layouts"

# Kitty
RM "$HOME/.config/kitty/kitty.conf"

# Rofi
RM "$HOME/.config/rofi/dracula.rasi"

# Polybar
RM "$HOME/.config/polybar/config.ini"
RM "$HOME/.config/polybar/launch.sh"

# i3status
RM "$HOME/.config/i3status/config"

# Shell customizations
RM "$HOME/.bashrc.d/10-color-prompt.bash"
RM "$HOME/.bashrc.d/20-dircolors-dracula.bash"
RM "$HOME/.bashrc.d/ros2_docker_aliases.bash"

# Docker/ROS2
RM "$HOME/docker-compose.ros2.yml"
RM "$HOME/ros2_ws/config/container_aliases.bash"

# Utility scripts
if [[ -d "$HOME/.local/bin" ]]; then
  for script in autotiling cool-login i3-autostart-2wins.sh i3-dashboard-2x2 \
                i3-smart-tiling.sh i3-split-two left-neofetch.sh monitoggle \
                polybar-cpu-temp.sh right-pipes.sh start-picom-safe.sh \
                uconsole-startup.sh wait-for-i3.sh; do
    RM "$HOME/.local/bin/$script"
  done
fi

# Wallpapers
RM "$HOME/.local/share/wallpapers"

# Font (optional - prompt user)
echo ""
read -p "Remove JetBrainsMono Nerd Font? (y/N) " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
  RM "$HOME/.local/share/fonts/JetBrainsMonoNerdFont-Regular.ttf"
  RM "$HOME/.local/share/fonts/JetBrainsMonoNerdFont-Bold.ttf"
  fc-cache -f
  echo "[✓] Font removed and cache updated"
fi

### Clean up empty directories
rmdir "$HOME/.config/i3" 2>/dev/null || true
rmdir "$HOME/.config/kitty" 2>/dev/null || true
rmdir "$HOME/.config/rofi" 2>/dev/null || true
rmdir "$HOME/.config/polybar" 2>/dev/null || true
rmdir "$HOME/.config/i3status" 2>/dev/null || true
rmdir "$HOME/.bashrc.d" 2>/dev/null || true
rmdir "$HOME/ros2_ws/config" 2>/dev/null || true
rmdir "$HOME/ros2_ws" 2>/dev/null || true

### Manual cleanup instructions
cat <<'EOF'

╔═══════════════════════════════════════════════════════════════════╗
║                  ✅ Uninstallation Complete!                      ║
╚═══════════════════════════════════════════════════════════════════╝

Configuration files have been removed.
Backup files (.bak.*) are preserved in their original locations.

Manual cleanup (if desired):
  1. Remove packages:
     sudo apt-get remove i3 polybar kitty rofi picom feh

  2. Remove Docker:
     sudo apt-get remove docker-ce docker-ce-cli containerd.io

  3. Remove user from docker group:
     sudo gpasswd -d $USER docker

  4. Clean up ~/.bashrc:
     Edit ~/.bashrc and remove the ~/.bashrc.d sourcing block

  5. Remove backup files:
     find ~ -name "*.bak.*" -type f -delete

EOF
