#!/bin/bash

SESSION=$USER

tmux -2 new-session -d -s $SESSION
# Setup a window for tailing log files
tmux new-window -t $SESSION:0 -n 'review_web_services'
tmux new-window -t $SESSION:1 -n 'ssh tunnel'
#tmux new-window -t $SESSION:2 -n 'linda_robot'
#tmux new-window -t $SESSION:3 -n 'linda_mapping_move_base'
#tmux new-window -t $SESSION:4 -n 'linda_people_perception'
#tmux new-window -t $SESSION:5 -n 'linda_bbc'

tmux select-window -t $SESSION:0
tmux send-keys "roslaunch y1_interfaces NUC.launch"

tmux select-window -t $SESSION:1
tmux send-keys "autossh -L8090:localhost:8090 hydro-default@scitosstrands"


# Set default window
tmux select-window -t $SESSION:0

# Attach to session
tmux -2 attach-session -t $SESSION

tmux setw -g mode-mouse on
