#!/bin/bash

SESSION="robot"
WS="$HOME/robot_ws"

COMMON="cd $WS && source /opt/ros/\$ROS_DISTRO/setup.bash && source $WS/install/setup.bash"

# Jezeli sesja juz istnieje, tylko sie do niej dolacz.
if tmux has-session -t "$SESSION" 2>/dev/null; then
    tmux attach -t "$SESSION"
    exit 0
fi

# ============================================================
# 1. SERIAL BRIDGE
# ============================================================

tmux new-session -d \
    -s "$SESSION" \
    -n serial \
    "$COMMON && ros2 run robot_motion serial_trajectory_bridge"

# ============================================================
# 2. ROBOT STATE PUBLISHER
# ============================================================

tmux new-window \
    -t "$SESSION" \
    -n state \
    "$COMMON && ros2 launch robot_description robot_state_publisher.launch.py"

# ============================================================
# 3. MANIPULATOR PRESET
# ============================================================

tmux new-window \
    -t "$SESSION" \
    -n preset \
    "$COMMON && ros2 run robot_motion manipulator_preset_node"

# ============================================================
# 4. MOVEIT
# ============================================================

tmux new-window \
    -t "$SESSION" \
    -n moveit \
    "$COMMON && ros2 launch robot_moveit_config move_group.launch.py"

# ============================================================
# 5. IK TARGET
# ============================================================

tmux new-window \
    -t "$SESSION" \
    -n ik \
    "$COMMON && ros2 run robot_motion ik_target_node"

# ============================================================
# 6. LEFT CAMERA
# ============================================================

tmux new-window \
    -t "$SESSION" \
    -n cam_left \
    "$COMMON && ros2 run camera_ros camera_node \
    --ros-args \
    -r __node:=left_camera \
    -r __ns:=/camera_left \
    -p camera:='/base/axi/pcie@120000/rp1/i2c@80000/imx708@1a' \
    -p role:=video \
    -p format:=RGB888 \
    -p width:=480 \
    -p height:=270 \
    -p FrameDurationLimits:='[200000,200000]'"

# ============================================================
# 7. RIGHT CAMERA
# ============================================================

tmux new-window \
    -t "$SESSION" \
    -n cam_right \
    "$COMMON && ros2 run camera_ros camera_node \
    --ros-args \
    -r __node:=right_camera \
    -r __ns:=/camera_right \
    -p camera:='/base/axi/pcie@120000/rp1/i2c@88000/imx708@1a' \
    -p role:=video \
    -p format:=RGB888 \
    -p width:=480 \
    -p height:=270 \
    -p FrameDurationLimits:='[200000,200000]'"

# ============================================================
# 8. STEREO DEPTH
# ============================================================

tmux new-window \
    -t "$SESSION" \
    -n stereo \
    "$COMMON && ros2 run camera_bridge stereo_depth"

# ============================================================
# 9. AI CUBE DETECTOR + VENV
# ============================================================

tmux new-window \
    -t "$SESSION" \
    -n ai_cube \
    "$COMMON && source $WS/venv_ai/bin/activate && ros2 run camera_bridge ai_cube_detector"

# ============================================================
# 10. ROSBRIDGE
# ============================================================

tmux new-window \
    -t "$SESSION" \
    -n rosbridge \
    "$COMMON && ros2 launch rosbridge_server rosbridge_websocket_launch.xml"

# ============================================================
# START NA PIERWSZYM OKNIE
# ============================================================

tmux select-window \
    -t "$SESSION":serial

tmux attach \
    -t "$SESSION"