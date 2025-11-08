# Persistent ROS2 in-container aliases
# Auto-sourced via /etc/profile.d/ros2-aliases.sh
if [ -f /opt/ros/humble/setup.bash ]; then
  . /opt/ros/humble/setup.bash
fi
alias rlist='ros2 topic list'
alias rhz='ros2 topic hz'      # usage: rhz /topic_name
alias recho='ros2 topic echo'  # usage: recho /topic_name
alias rnodes='ros2 node list'
alias rinfo='echo Nodes:; ros2 node list; echo; echo Topics:; ros2 topic list'
rgrep(){ ros2 topic list | grep -i "$1" ; }
export -f rgrep 2>/dev/null || true
