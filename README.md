## sp_scenario_2

```
mkdir -p ~/sp_scenario_2_ws/src
cd ~/sp_scenario_2_ws
wget https://raw.githubusercontent.com/sequenceplanner/sp_scenario_2/master/ros2.repos
vcs import src < ros2.repos
. /opt/ros/galactic/setup.bash 
colcon build --cmake-args -DCARGO_CLEAN=ON
```

## quickstart
with the real robot or actual ursim:
```
. /opt/ros/galactic/setup.bash
. ~/sp_scenario_2_ws/install/setup.bash
ros2 launch sp_scenario_2 bringup.launch.py
```

with the simple robot simulator:
```
. /opt/ros/galactic/setup.bash
. ~/sp_scenario_2_ws/install/setup.bash
ros2 launch sp_scenario_2 bringup_simple.launch.py
```

## get a robot or a ursim simulator:

1. Get docker: \
   https://docs.docker.com/get-docker/ \
   NOTE: At least version 19.\
   NOTE: Turn off VPN during installation.
2. Get and start the URSim docker container:

   ```
   sudo docker run --rm -it \
   --name="dockursim" \
   -e ROBOT_MODEL=UR3 \
   -p 5901:5901 \
   -p 6080:6080 \
   -p 29999:29999 \
   -p 30001-30004:30001-30004 \
   --cpus=2 \
   --privileged kristoferb/ursim:latest
   ```
3. The Universal Robot Interface can now be accessed at: \
 http://localhost:6080/vnc.html?host=localhost&port=6080.
4. To start the container again, you will probably have to kill and remove it first:

   ```
   sudo docker kill dockursim
   sudo docker rm dockursim
   ```

## Todo describe scenario in detail...
sp_scenario_2
