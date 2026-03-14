#!/bin/bash

SESSION=bot_bringup

tmux kill-session -t $SESSION 2>/dev/null
tmux new-session -d -s $SESSION

tmux send-keys -t $SESSION "export ROS_DOMAIN_ID=0" C-m
tmux send-keys -t $SESSION "ros2 run nana_core apriltag_cameraPI_pub" C-m

sleep 2

tmux split-window -h -t $SESSION
tmux send-keys -t $SESSION "export ROS_DOMAIN_ID=0" C-m
tmux send-keys -t $SESSION "ros2 run nana_core plot_cameraPI_pub" C-m

sleep 2

tmux split-window -h -t $SESSION
tmux send-keys -t $SESSION "export ROS_DOMAIN_ID=0" C-m
tmux send-keys -t $SESSION "ros2 run nana_core Orchestrator_node" C-m

sleep 2

tmux split-window -h -t $SESSION
tmux send-keys -t $SESSION "export ROS_DOMAIN_ID=0" C-m
tmux send-keys -t $SESSION "ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200 -v6" C-m

tmux select-layout -t $SESSION even-horizontal

tmux attach -t $SESSION

