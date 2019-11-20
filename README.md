# VDM (visualization-of-dynamical-model)

RoboCup project visualizing acting forces on arbitrary urdf-robot-models in
rviz.

## Requirments

- rviz
- ros

## Getting Started

### Setup

```zsh
mkdir visual-dynamic-model
cd visual-dynamic-model
mkdir src
cd src
git clone git@github.com:nalchevanidze/vdm.git
cd ../

catkin init
# adds env variables to your terminal
source /opt/ros/kinetic/setup.<zsh | sh .... >
catkin build
source devel/setup.<zsh | sh .... >
```

if you want not to set env variables every time you open the shell, you can write it in your `.<bash | zsh >rc`

```bash
source /opt/ros/kinetic/setup.bash
source [catkin_project_dir]/devel/setup.bash
```

## Launch

Change into `src/vdm` and execute your desired launch-file:

```bash
roslaunch launch/[name].launch
```

## Known Problems

It may be possible that rviz does not properly visualize the geometry of your
robot model (it might for example be invisible).
According to [the ROS forum](https://answers.ros.org/question/271357/rviz-doesnt-show-any-shape/) it can be fixed by adding the following line to your `.bashrc`:

```bash
export LC_NUMERIC="en_US.UTF-8"
```

## About

### Team

VDM is written and maintained by:

- [nalchevanidze](https://github.com/nalchevanidze)
- [haukesomm](https://github.com/haukesomm)
