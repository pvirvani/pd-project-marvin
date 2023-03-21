# vision_pacbot

Vision module for PacBot project

## Getting started

This module to be installed on Jetson Module, and supposed to Work with RealSense L515 RGBD camera.


## Operation

```sh
# 1. clone this repo inside src folder
$ git clone https://gricad-gitlab.univ-grenoble-alpes.fr/pacbot_group/vision_pacbot.git ./src/

# 2. build the workspace
$ catkin init
$ catkin build

# 3. source the workspace
$ . devel/setup.bash

# 4. launch the main launch file, patterns are 'simple' or 'complex'
$ roslaunch vision_launch vision.launch pattern:="complex"

# 5. run the vision logic in another terminal (after sourcing [step 3] )
$ . devel/setp.bash
$ roslaunch vision_logic vision_logic.launch pattern:="complex" 
```
