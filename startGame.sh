#!/bin/bash
SESSION='BALLGAME'
TURTLEBOT='roslaunch turtlebot_bringup minimal.launch'
KINECT='roslaunch openni_launch openni.launch'

tmux -2 new-session -d -s $SESSION
echo "Started Session $SESSION"

#tmux new-window -t $SESSION:0 -n 'Bringup'
tmux split-window -h
tmux select-pane -t 0
tmux send-keys "$TURTLEBOT" C-m
tmux select-pane -t 1
tmux send-keys "$KINECT" C-m


tmux new-window -t $SESSION:1 -n 'Nodes'
i=0
while read p; do
  tmux split-window -v
  tmux select-pane -t $i
  echo "Starting Node $p" 
  tmux resize-pane -D 5
  tmux send-keys "python $p" C-m
  i=$((i+1))
done <Nodes.txt
tmux select-pane -t $i
tmux send-keys "tmux kill-session -t $SESSION"
echo "To Kill All Running Programms type:"
echo "   tmux kill-session -t $SESSION"
sleep 5

tmux attach -t $SESSION
