# Protocol

## Week 1-2 

- First demo-implementation of a moving robot in Python 


## Week 3-4

- Migration to C++ and moveit


## Week 4-9

- Implementation of a simple jacobian-calculator-class
- Encountered problems while obtaining the velocities from moveit since we 
  didn't know how to properly interprete the results
- Migration to a `Subscriber`-based implementation


## Week 10

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