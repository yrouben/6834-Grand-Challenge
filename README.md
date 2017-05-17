# 6.834/16.412 Grand Challenge - Sampling-based Path Planning Team

### To run simulation:
Create a directory titled `catkin_ws`. Clone this repo into that directory and rename the repo to `src`. Using `sudo apt-get install`, install the following dependencies:

* ros-kinetic-costmap-coverter
* python-networkx
* python-shapely
* python-descartes

(note: this assumes you have ROS installed already.)

Then, from the `catkin_ws` directory, run:
```sh
~/catkin_ws$ catkin_make
~/catkin_ws$ roscore

# new terminal
~/catkin_ws$ roslaunch cogrob grand_challenge_sim.launch

# new terminal
~/catkin_ws$ rosrun cogrob achieve-sim-goals
```

To edit the locations that the robot travels to, modify `cogrob/data/sim-map.yaml`.


