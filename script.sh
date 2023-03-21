#!/bin/bash

echo "Running Project PD" &
sleep 0.2 
xterm -e "roscore" &
sleep 3
xterm -e "roslaunch rosbridge_server rosbridge_websocket.launch --wait" &
sleep 1 
xterm -e "cd ~/pardeep_files/pd-project-modules/GUI_pacbot/ && npm run serve" &
sleep 0.5
xterm -e "cd ~/pardeep_files/pd-project-modules/project_marvin/robot-backend/ && source env/bin/activate && python projects_app.py" &
sleep 0.5
xterm -e "cd ~/pardeep_files/pd-project-modules/project_marvin/robot-frontend/ && quasar dev"
