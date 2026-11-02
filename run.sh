#!/usr/bin/env bash
###############################################################################
# TurtleBot3 Nav2 Mission Control - One-Click Launcher
###############################################################################
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

MODEL="waffle"
HEADLESS="false"
PORT="8000"
WEB_ONLY="false"
NO_BROWSER="false"

print_usage() {
  echo "Usage: ./run.sh [OPTIONS]"
  echo ""
  echo "Options:"
  echo "  --model <burger|waffle|waffle_pi>   Select TurtleBot3 model (default: waffle)"
  echo "  --headless                         Run Gazebo headless without 3D GUI window"
  echo "  --port <PORT>                      Web server port (default: 8000)"
  echo "  --web-only                         Launch only the local Web Server node"
  echo "  --no-browser                       Do not auto-open the browser tab"
  echo "  --build                            Rebuild workspace packages before launching"
  echo "  --help                             Show this help message"
  echo ""
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --model)
      MODEL="$2"
      shift 2
      ;;
    --headless)
      HEADLESS="true"
      shift
      ;;
    --port)
      PORT="$2"
      shift 2
      ;;
    --web-only)
      WEB_ONLY="true"
      shift
      ;;
    --no-browser)
      NO_BROWSER="true"
      shift
      ;;
    --build)
      DO_BUILD="true"
      shift
      ;;
    --help|-h)
      print_usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1"
      print_usage
      exit 1
      ;;
  esac
done

export TURTLEBOT3_MODEL="$MODEL"

echo "================================================================="
echo "   🤖 TURTLEBOT3 NAV2 MISSION CONTROL SYSTEM                   "
echo "================================================================="
echo "  • ROS 2 Distro: Humble"
echo "  • Robot Model : $TURTLEBOT3_MODEL"
echo "  • Headless Sim: $HEADLESS"
echo "  • Web Server  : http://localhost:$PORT"
echo "================================================================="

# Clean up any orphaned Gazebo or Nav2 instances from previous runs
if pgrep -x "gzserver" > /dev/null 2>&1 || pgrep -x "controller_server" > /dev/null 2>&1; then
  echo "⚠️  Cleaning up previous background simulation/nav2 instances..."
  killall -9 gzserver gzclient controller_server smoother_server planner_server behavior_server bt_navigator 2>/dev/null || true
  sleep 1
fi

# Source ROS 2 Humble
if [ -f /opt/ros/humble/setup.bash ]; then
  source /opt/ros/humble/setup.bash
else
  echo "❌ Error: ROS 2 Humble was not found at /opt/ros/humble/setup.bash"
  exit 1
fi

# Ensure local user model cache is set up
mkdir -p "$HOME/.gazebo/models"
if [ ! -d "$HOME/.gazebo/models/sun" ] && [ -d "/usr/share/gazebo-11/models/sun" ]; then
  cp -rn /usr/share/gazebo-11/models/* "$HOME/.gazebo/models/" 2>/dev/null || true
fi
if [ ! -d "$HOME/.gazebo/models/turtlebot3_world" ] && [ -d "/opt/ros/humble/share/turtlebot3_gazebo/models" ]; then
  cp -rn /opt/ros/humble/share/turtlebot3_gazebo/models/* "$HOME/.gazebo/models/" 2>/dev/null || true
  rm -rf "$HOME/.gazebo/models/turtlebot3_autorace_2020" 2>/dev/null || true
fi

# Sanitize GAZEBO environment to prevent Gazebo GUI hang:
# 1. Disable dead online model database http://models.gazebosim.org
export GAZEBO_MODEL_DATABASE_URI=""
# 2. Strip /opt/ros/humble/share which depthai_descriptions injects, causing Gazebo to scan 500+ ROS packages
CLEAN_MODEL_PATH=$(echo "$GAZEBO_MODEL_PATH" | tr ':' '\n' | grep -v -E "^/opt/ros/humble/share/?$" | tr '\n' ':' | sed 's/:$//')
export GAZEBO_MODEL_PATH="$HOME/.gazebo/models:/usr/share/gazebo-11/models:/opt/ros/humble/share/turtlebot3_gazebo/models:${CLEAN_MODEL_PATH}"
export GAZEBO_RESOURCE_PATH="/usr/share/gazebo-11:/opt/ros/humble/share/turtlebot3_gazebo:${GAZEBO_RESOURCE_PATH:-}"

# Build if requested or if install directory does not exist
if [ "$DO_BUILD" = "true" ] || [ ! -d "$SCRIPT_DIR/install" ]; then
  echo "📦 Building workspace packages..."
  colcon build --symlink-install
fi

# Source workspace overlay
source "$SCRIPT_DIR/install/setup.bash"

# Auto-open browser in background after 3.5 seconds
if [ "$NO_BROWSER" != "true" ] && [ -n "$DISPLAY" ] && command -v xdg-open > /dev/null 2>&1; then
  (
    sleep 3.5
    echo "🌐 Opening Mission Control Dashboard in browser..."
    xdg-open "http://localhost:$PORT" > /dev/null 2>&1 || true
  ) &
fi

echo ""
echo "🚀 Launching system components..."
echo "👉 Web Dashboard URL: http://localhost:$PORT"
echo ""

if [ "$WEB_ONLY" = "true" ]; then
  ros2 launch turtlebot3_nav2_web web_server.launch.py port:="$PORT"
else
  ros2 launch turtlebot3_nav2_web full_system.launch.py \
    model:="$TURTLEBOT3_MODEL" \
    headless:="$HEADLESS" \
    port:="$PORT"
fi
