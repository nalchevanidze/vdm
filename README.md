# Visualization-of-dynamical-model

Uni hamburg project Robocup

## requirments

- rviz
- ros

## get started

### setup

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

if you want not to set env variables every time you open the shell, you can write it in your

###### .<bash | zsh >rc

```bash
source /opt/ros/kinetic/setup.bash
source ~/{catkin_project_dir}/devel/setup.bash
```

# launch

go to: 'src/vdm'

```bash
roslaunch launch/line_rviz.launch
```

## About

### team

vdm is written and maintained by

- [nalchevanidze](https://github.com/nalchevanidze)
- [haukesomm](https://github.com/haukesomm)
