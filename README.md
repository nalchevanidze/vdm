# VDM (visualization-of-dynamical-model)

RoboCup project visualizing acting forces on arbitrary urdf-robot-models in
rviz.

## Requirments

- rviz
- ros

## Getting Started

### Setup

#### Workspace

```zsh
mkdir vdm
cd vdm
mkdir src
cd src
git clone git@github.com:nalchevanidze/vdm.git
git clone git@github.com:bit-bots/bitbots_meta.git

cd ../
catkin init
```

#### .bashrc

In order to develop for ros using our vdm-package, your environment needs to
be set up accordingly.  
Please find below an **example** configuration:

```bash
# Configure VDM using environment variables:

# ROS-version codename (kinetic, melodic, etc.)
export VDM_ROS_VERSION="melodic"
# catkin-workspace path
export VDM_WS_PATH="$HOME/catkin_ws"
# path of the cloned bitbots_meta repository
export VDM_BITBOTS_META_PATH="$VDM_WS_PATH/src/bitbots_meta"


# Setup the ros-specific environment:

source /opt/ros/kinetic/setup.bash
source $VDM_WS_PATH/devel/setup.bash
```

## Launch

Change into your catkin-workspace and execute your desired launch-file:

```bash
# general:
roslaunch vdm [lauchfile-name]

# jacobian-testing:
rosrun vdm jacobian
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

