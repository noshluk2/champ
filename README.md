
## 1. Installation

- Clone and install all dependencies:
```
    sudo apt install -y python3-rosdep
    rosdep update

    mkdir -p dogbot_ws/src
    cd dogbot_ws/src
    git clone --recursive https://github.com/noshluk2/champ.git
    git clone https://github.com/chvmp/champ_teleop -b ros2
    cd ..
    rosdep install --from-paths src --ignore-src -r -y
```
## Run Simulation
- Build your workspace:
```
    cd dogbot_ws
    colcon build
    source ~/dogbot_ws/install/setup.bash
```
- Gazebo + Rviz2
```
 ros2 launch champ_config gazebo_gen.launch.py
```
- Perform Path planning
```bash
    ros2 run champ_navigation path_planner
```
- Drive Robot
```
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
```