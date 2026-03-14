#!/bin/bash
SESSION=bot_bringup
tmux kill-session -t $SESSION 2>/dev/null
tmux new-session -d -s $SESSION

# Pane 0: AprilTag Camera
tmux send-keys -t $SESSION "export ROS_DOMAIN_ID=0" C-m
tmux send-keys -t $SESSION "ros2 run nana_core apriltag_cameraPI_pub" C-m
sleep 2

# Pane 1: Plot Camera
tmux split-window -h -t $SESSION
tmux send-keys -t $SESSION "export ROS_DOMAIN_ID=0" C-m
tmux send-keys -t $SESSION "ros2 run nana_core plot_cameraPI_pub" C-m
sleep 2

# Pane 2: micro-ROS Agent #1 → ttyUSB0
tmux split-window -h -t $SESSION
tmux send-keys -t $SESSION "export ROS_DOMAIN_ID=0" C-m
tmux send-keys -t $SESSION "ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200 -v6" C-m
sleep 2

# Pane 3: micro-ROS Agent #2 → ttyUSB1
# *** แก้ไข: ลบ typo ภาษาไทย + เพิ่ม --ros-args -r __ns:=/agent2 ***
tmux split-window -h -t $SESSION
tmux send-keys -t $SESSION "export ROS_DOMAIN_ID=0" C-m
tmux send-keys -t $SESSION "ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB1 -b 115200 -v6 --ros-args -r __ns:=/agent2" C-m

tmux select-layout -t $SESSION even-horizontal
tmux attach -t $SESSION