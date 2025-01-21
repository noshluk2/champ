
## 1. Installation

- Clone and install all dependencies:
```
    sudo apt install -y python3-rosdep
    rosdep update

    cd <your_ws>/src
    git clone --recursive https://github.com/noshluk2/champ.git
    git clone https://github.com/chvmp/champ_teleop -b ros2
    cd ..
    rosdep install --from-paths src --ignore-src -r -y
```
- Build your workspace:
```
    mkdir -p dog_robot/src
    cd dog_robot
    colcon build
    source dog_robot/install/setup.bash
```
## Run Simulation
- RVIZ:
```
    ros2 launch champ_config bringup.launch.py rviz:=true
```
- Drive Robot
```
    ros2 launch champ_teleop teleop.launch.py
```
