# _panda_interfaces_  

The _panda_interfaces_ package contains all the messages, services and actions used in the project. Most of them are used in the _panda_utils_ package for: 

- trajectory generation
- sensors info
- human-robot interaction
- debugging and logging

## Actions  

### Trajectory generation    

The actions aimed to the generation of trajectories for the robot are: 
- `CartTraj`
- `LoopCartTraj`
- `StopTraj`
- `JointTraj`

All the actions are very similar to each other and really simple in the definition. The implementation are provided in the [*panda_utils*](./panda_utils.md) package.  

## Messages  

### Commands   

The messages used to send commands to the various components of the project (controller, lower level bridges) are:

- `CartesianCommand`: used to send cartesian trajectories to the controller
- `JointsCommand`
- `JointsEffort`

The joint level commands are used in an *intermediate inverse dynamics controller*, solely used to bring the panda robot in a *well conditioned* configuration before activating the impedance controller (and deactivating the inverse dyn one).  

### Measures  

The messages used for sensor measures and operator relative alerts are: 

- `HumanContact`
- `JointTorqueMeasure`
- `JointTorqueMeasureStamped`

The `HumanContact` is used to indicate which one of the wrist of the operator is in contact, if any, with one of the robot's frames. The other 2 messages are used to publish the external joint torque measures of the simulated panda robot in the gazebo environment.  

### Debug  

The rest of the messages are used for the publication of debug messages: 

- `DoubleStamped`
- `DoubleArrayStamped`
- `BodyLengths`
- `JointsPos`

## Services  

The package contains a single service, `SetComplianceMode`, similar to the `SetBool` service, used to switch the controller mode from the position control to the compliance control mode. 
