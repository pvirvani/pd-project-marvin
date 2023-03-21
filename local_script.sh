#!/bin/bash

echo "Running Project PD" &
sleep 0.5
xterm -e "cd ~/pardeep_files/pd-project-modules/GUI_pacbot/ && npm run serve" &
sleep 0.5
xterm -e "cd ~/pardeep_files/pd-project-modules/project_marvin/robot-backend/ && source env/bin/activate && python projects_app.py" &
sleep 0.5
xterm -e "cd ~/pardeep_files/pd-project-modules/project_marvin/robot-frontend/ && quasar dev"


# echo "Running Project PD" &
# sleep 0.5
# gnome-terminal -- bash -c "cd ~/pardeep_files/pd-project-modules/GUI_pacbot/ && npm run serve; exec bash" &
# sleep 0.5
# gnome-terminal -- bash -c  "cd ~/pardeep_files/pd-project-modules/project_marvin/robot-backend/ && source env/bin/activate && python projects_app.py; exec bash" &
# sleep 0.5
# gnome-terminal -- bash -c "cd ~/pardeep_files/pd-project-modules/project_marvin/robot-frontend/ && quasar dev; exec bash"
