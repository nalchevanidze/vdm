# Protocol

## week 1

- orga
- we found each other

## week 2 (23.10.2019)

- we created github repo
- decided which project to work on
- started to prepare our local environments, With Docker, or ...
  - david: on mac didnot workded at all
  - hauke: after 2 week of torture, he finally did with VirtualBox

## Week 3 (30.10.2019)

- decided to work with uni computers
- we did research: which language, how ros works at all ...
- we choose `python`
- setup project with `catkin`
- First demo-implementation of a moving robot in Python
- elaborated `rviz` and configured `*.launch` files.
- created `main.launch`

## Week 4 (6.11.2019)

- elaborated `urdf`
- created `line.urdf` TODO: screenshot
- implemented `LineJointStatePublisher` with python, this publishes publishes sine values, to which the line model subscribes and moves accordingly
- thought on how to obtain list of joints from the urdf

## Week 5 (13.11.2019)

- we merged the different nodes in one combined  `main.launch`
- created sofisticated model of tree
- we found `moveit`.

## Week 6 (20.11.2019)

- we decided to move `c++` (worst decision)
- we tried to setup moveit `tree.urdf`. could not succeced
  could not generate model description on our own.

## Week 7 (27.11.2019)

- Migration to C++ and moveit
- we setup `movit` with model of `wolfgang`
- wrote first succesfull jacobian calculator. that:
  - lists all chained move groups
  - prints out jacobian value for each joint

## week 8 (04.12.2019)

- implemented `RobotMarkerPublisher`, that for every joint
  - logs jackobians
  - displays `ARROW` shape

## Week 9 (11.12.2019)

- Split up code into classes
- deleted python code

## week 10 (18.12.2019)

- for each joint publisher publishes jacobian value on marker.v TODO: screenshot
- Encountered problems while obtaining the velocities from moveit since we
  didn't know how to properly interprete the results

## Week 11 (8.1.2020)

- Migration to a `Subscriber`-based implementation

## week 12 (15.1.2020)

- Implemented functionality to listen to the 'joint_states' topic and publish markers as a reaction
- topic `joint_states` does not provide velocities , it returns just an empty array.
- Current implementation is ugly and needs to be put in a separate class
- A simple static marker is published as a proof of concept

TODO:
- Create marker based on the actual velocity as in the previous AbstractMarkerPublisher class
  - Don't re-use AbstractMarkerPublisher and the related classes, it will probably be easier to re-write it
- try out to caclulate velocity from joint position
  - try out to get positions by JointState
- prepare for presentation

## week 13 (21.1.2020)

- fullfill protocol
- start presentation

## week 14 (22.1.2020)

- Tried `tf2` as an alternative approach for velocity calculation,
 `tf2` provides absolute velocities. so we don't need jacobian  anymore
- with python script we mocked `joint_state`, so we can have velocities
  for calculations
- we made both approaches simultaneously
