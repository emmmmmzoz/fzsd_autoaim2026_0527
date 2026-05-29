#!/usr/bin/env bash
set -euo pipefail

# ==============================================================================
# One-Click Video Test Launcher for AutoAim
# ==============================================================================
# Usage:
#   ./scripts/run_video_test.sh --video test1 --loop true --fps 30 --plot true
#   ./scripts/run_video_test.sh --video test2 --loop true --fps 30 --plot true
#   ./scripts/run_video_test.sh --video test3 --loop false --fps 30 --plot true
# ==============================================================================

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

log_info()  { echo -e "${GREEN}[INFO]${NC}  $(date '+%H:%M:%S') $*"; }
log_warn()  { echo -e "${YELLOW}[WARN]${NC}  $(date '+%H:%M:%S') $*"; }
log_error() { echo -e "${RED}[ERROR]${NC} $(date '+%H:%M:%S') $*"; }
log_step()  { echo -e "${BLUE}[STEP]${NC}  $(date '+%H:%M:%S') $*"; }

# ---- defaults ----
VIDEO="test1"
LOOP="true"
FPS="30.0"
SCALE="1.0"
USE_SERIAL="false"
PLOT="false"
PLOT_WINDOW="20.0"
TMUX_MODE="auto"

# ---- parse args ----
while [[ $# -gt 0 ]]; do
    case $1 in
        --video)     VIDEO="$2"; shift 2 ;;
        --loop)      LOOP="$2"; shift 2 ;;
        --fps)       FPS="$2"; shift 2 ;;
        --scale)     SCALE="$2"; shift 2 ;;
        --use-serial) USE_SERIAL="$2"; shift 2 ;;
        --plot)      PLOT="$2"; shift 2 ;;
        --plot-window) PLOT_WINDOW="$2"; shift 2 ;;
        --tmux)      TMUX_MODE="$2"; shift 2 ;;
        -h|--help)
            echo "Usage: $0 [OPTIONS]"
            echo "  --video NAME       Video name: test1, test2, test3 (default: test1)"
            echo "  --loop BOOL        Loop video playback (default: true)"
            echo "  --fps NUM          Publish frame rate (default: 30.0)"
            echo "  --scale NUM        Resize factor, 0=auto (default: 1.0)"
            echo "  --use-serial BOOL  Launch serial driver (default: false)"
            echo "  --plot BOOL        Launch angle error plotter (default: false)"
            echo "  --plot-window NUM  Plot rolling window in seconds (default: 20.0)"
            echo "  --tmux BOOL        Use tmux multi-window (default: auto)"
            exit 0
            ;;
        *) log_error "Unknown option: $1"; exit 1 ;;
    esac
done

# ---- find workspace root ----
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

log_info "============================================"
log_info "  AutoAim Video Test Launcher"
log_info "============================================"
log_info "Workspace:  $WORKSPACE_ROOT"
log_info "Video:      $VIDEO"
log_info "Loop:       $LOOP"
log_info "FPS:        $FPS"
log_info "Scale:      $SCALE"
log_info "Use Serial: $USE_SERIAL"
log_info "Plot:       $PLOT"
log_info "============================================"

# ---- pre-flight checks ----
log_step "Running pre-flight checks..."

# Source ROS2
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
elif [ -n "${ROS_DISTRO:-}" ] && [ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
    source "/opt/ros/$ROS_DISTRO/setup.bash"
else
    log_error "Cannot find ROS2 setup.bash. Set ROS_DISTRO or source ROS2 manually."
    exit 1
fi

# Source workspace overlay
if [ -f "$WORKSPACE_ROOT/install/setup.bash" ]; then
    source "$WORKSPACE_ROOT/install/setup.bash"
else
    log_error "install/setup.bash not found. Run 'colcon build --symlink-install' first."
    exit 1
fi

# Check packages
check_pkg() {
    if ros2 pkg list 2>/dev/null | grep -q "^$1$"; then
        log_info "Package '$1' found"
    else
        log_error "Package '$1' NOT found. Build it first."
        return 1
    fi
}
check_pkg video_to_ros
check_pkg rm_vision_bringup

# Check video file exists
_VIDEO_FILE=""
for ext in .mp4 .avi .mkv .mov .webm .flv .wmv; do
    candidate="$WORKSPACE_ROOT/videos/$VIDEO$ext"
    if [ -f "$candidate" ]; then
        _VIDEO_FILE="$candidate"
        break
    fi
done
# Also try as-is
if [ -z "$_VIDEO_FILE" ] && [ -f "$WORKSPACE_ROOT/videos/$VIDEO" ]; then
    _VIDEO_FILE="$WORKSPACE_ROOT/videos/$VIDEO"
fi

if [ -z "$_VIDEO_FILE" ]; then
    log_error "Video file not found: videos/$VIDEO.*"
    log_error "Searched in: $WORKSPACE_ROOT/videos/"
    ls -la "$WORKSPACE_ROOT/videos/" 2>/dev/null || echo "(videos/ dir empty or missing)"
    exit 1
fi
log_info "Video file: $_VIDEO_FILE"

# Check camera_info.yaml
CI_PATH="$WORKSPACE_ROOT/src/rm_vision/rm_vision_bringup/config/camera_info.yaml"
if [ -f "$CI_PATH" ]; then
    log_info "CameraInfo YAML: $CI_PATH"
else
    log_error "CameraInfo YAML not found at $CI_PATH"
    exit 1
fi

# Check that no real camera node is running
if ros2 topic list 2>/dev/null | grep -q '/image_raw'; then
    log_warn "/image_raw topic already exists — a camera node may be running"
    log_warn "Consider stopping it before launching video test"
fi

log_info "All pre-flight checks passed."

# ---- cleanup on exit ----
PIDS=()
cleanup() {
    log_info "Cleaning up..."
    for pid in "${PIDS[@]}"; do
        kill "$pid" 2>/dev/null || true
    done
    # Kill any remaining ros2 launch or plotter in our process group
    jobs -p 2>/dev/null | xargs -r kill 2>/dev/null || true
    log_info "Cleanup done."
}
trap cleanup EXIT INT TERM

# ---- tmux or background launch ----
HAS_TMUX=false
if [ "$TMUX_MODE" != "false" ] && command -v tmux &>/dev/null; then
    HAS_TMUX=true
fi

if $HAS_TMUX && [ "$TMUX_MODE" != "false" ]; then
    log_info "Launching with tmux multi-window mode..."
    SESSION="autoaim_video_$$"

    # Window 1: ros2 launch
    tmux new-session -d -s "$SESSION" -n launch
    tmux send-keys -t "$SESSION:launch" \
        "source /opt/ros/humble/setup.bash && source $WORKSPACE_ROOT/install/setup.bash && \
         ros2 launch rm_vision_bringup video_test.launch.py \
             video:=$VIDEO loop:=$LOOP fps:=$FPS scale:=$SCALE use_serial:=$USE_SERIAL" C-m

    # Window 2: topic hz for /image_raw
    sleep 2
    tmux new-window -t "$SESSION" -n topic_hz
    tmux send-keys -t "$SESSION:topic_hz" \
        "source /opt/ros/humble/setup.bash && source $WORKSPACE_ROOT/install/setup.bash && \
         echo 'Waiting for topics...' && sleep 3 && \
         ros2 topic hz /image_raw" C-m

    # Window 3: angle error plotter (if --plot true)
    if [ "$PLOT" = "true" ]; then
        tmux new-window -t "$SESSION" -n plotter
        tmux send-keys -t "$SESSION:plotter" \
            "source /opt/ros/humble/setup.bash && source $WORKSPACE_ROOT/install/setup.bash && \
             sleep 5 && \
             ros2 run video_to_ros angle_error_plotter --ros-args \
                 -p window_sec:=$PLOT_WINDOW" C-m
    fi

    # Window 4: topic echo /debug/angle_error
    if [ "$PLOT" = "true" ]; then
        tmux new-window -t "$SESSION" -n angle_echo
        tmux send-keys -t "$SESSION:angle_echo" \
            "source /opt/ros/humble/setup.bash && source $WORKSPACE_ROOT/install/setup.bash && \
             sleep 5 && \
             ros2 topic echo /debug/angle_error" C-m
    fi

    log_info "tmux session '$SESSION' started. Attach with: tmux attach -t $SESSION"
    log_info "Windows: launch | topic_hz | plotter | angle_echo"

    # Health checks after a brief wait
    sleep 6
    log_step "Post-launch health checks..."

    if ros2 topic list 2>/dev/null | grep -q '/image_raw'; then
        log_info "/image_raw topic is active"
    else
        log_warn "/image_raw topic not yet visible (may need more time)"
    fi

    if ros2 topic list 2>/dev/null | grep -q '/camera_info'; then
        log_info "/camera_info topic is active"
    else
        log_warn "/camera_info topic not yet visible"
    fi

    if [ "$PLOT" = "true" ]; then
        if ros2 topic list 2>/dev/null | grep -q '/debug/angle_error'; then
            log_info "/debug/angle_error topic is active"
        else
            log_warn "/debug/angle_error topic not yet visible (detector may not have found a target yet)"
        fi
    fi

    log_info "Press Ctrl+C to stop and cleanup, or attach: tmux attach -t $SESSION"
    # Wait for tmux session to end
    while tmux has-session -t "$SESSION" 2>/dev/null; do
        sleep 2
    done

else
    log_info "Launching with background processes (no tmux)..."

    # Launch ros2 launch in background
    ros2 launch rm_vision_bringup video_test.launch.py \
        video:=$VIDEO loop:=$LOOP fps:=$FPS scale:=$SCALE use_serial:=$USE_SERIAL &
    LAUNCH_PID=$!
    PIDS+=($LAUNCH_PID)
    log_info "Launch PID: $LAUNCH_PID"

    # Wait for topics to appear
    sleep 4

    # Topic hz in background
    ros2 topic hz /image_raw &
    HZ_PID=$!
    PIDS+=($HZ_PID)

    # Plotter
    if [ "$PLOT" = "true" ]; then
        sleep 2
        ros2 run video_to_ros angle_error_plotter --ros-args \
            -p window_sec:=$PLOT_WINDOW &
        PLOTTER_PID=$!
        PIDS+=($PLOTTER_PID)
        log_info "Plotter PID: $PLOTTER_PID"
    fi

    # Health checks
    sleep 2
    log_step "Post-launch health checks..."

    check_topic() {
        local t=$1
        if ros2 topic list 2>/dev/null | grep -q "$t"; then
            log_info "  $t: ACTIVE"
        else
            log_warn "  $t: not found"
        fi
    }

    ros2 topic list 2>/dev/null | grep -E '/(image_raw|camera_info|debug/angle_error|detector|tracker)' || true
    check_topic '/image_raw'
    check_topic '/camera_info'
    check_topic '/detector/armors'
    check_topic '/tracker/target'
    if [ "$PLOT" = "true" ]; then
        check_topic '/debug/angle_error'
    fi

    log_info "============================================"
    log_info "  Video test running. Press Ctrl+C to stop."
    log_info "  Video:      $_VIDEO_FILE"
    log_info "  Loop:       $LOOP"
    log_info "  FPS:        $FPS"
    log_info "  Real camera: NO (video_to_ros only)"
    log_info "  Plotter:    $PLOT"
    log_info "============================================"

    # Wait for launch to exit (or Ctrl+C)
    wait $LAUNCH_PID 2>/dev/null || true
fi
