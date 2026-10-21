#!/bin/bash
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SETUP_FILE="$WS_DIR/install/setup.bash"
if [ ! -f "$SETUP_FILE" ]; then
    echo "[ERROR] 未找到工作空间 setup 文件: $SETUP_FILE"
    echo "请先执行: cd $WS_DIR && colcon build --symlink-install"
    exit 1
fi
WSL_GPU_ENV=""
if grep -qi microsoft /proc/version 2>/dev/null; then
    WSL_GPU_ENV='export GALLIUM_DRIVER=d3d12; export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA;'
fi
open_terminal() {
    local title="$1"
    local cmd="$2"
    local full_cmd="${WSL_GPU_ENV} source $SETUP_FILE && $cmd; exec bash"
    if command -v gnome-terminal &>/dev/null; then
        gnome-terminal --title="$title" -- bash -c "$full_cmd" &
    elif command -v xterm &>/dev/null; then
        xterm -T "$title" -e bash -c "$full_cmd" &
    elif command -v konsole &>/dev/null; then
        konsole --new-tab -p tabtitle="$title" -e bash -c "$full_cmd" &
    else
        echo "[ERROR] 未找到可用的终端模拟器 (gnome-terminal / xterm / konsole)"
        exit 1
    fi
}
export QT_QPA_PLATFORM=xcb
open_terminal "tf2" "ros2 run tf2_ros static_transform_publisher 0 0.15 0 0 0 0 base_link livox_frame"
open_terminal "sp_nav" "ros2 launch sp_nav_bringup sp_nav.launch.py"
sleep 3
open_terminal "sim" "ros2 launch sp_nav_sim sim_robot.launch.py"
echo "所有节点已在新终端中启动。"
