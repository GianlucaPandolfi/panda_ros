# Robot controller

The [*impedance_controller* node](../overview/panda_utils.md#impedance_controller) can be easily reused to implement other control law, acting in joint or operational space. The focus will be on the 2 main functions of the node 
```cpp
on_configure(const rclcpp_lifecycle::State &)
...
on_activate(const rclcpp_lifecycle::State &)
```
allowing to characterize almost entirely the behaviour of the controller.  

## *on_configure*  

This function initialize almost all the parameters of the node, and depending on the mode of operation chosen for the controller, initialize the communication with the **Franka Emika panda** robot using the FCI.  
The first part of the function is used to initialize the common resources to both the modes: 
- the publishers used for debug, with a helper `DebugPublisher` class,
- the controller gains and control update frequency. 

After this section the simulation doesn't need additional setup.  

If we are communicating with the FCI we need to: 

- instantiate the communication, using the **robot ip**,
- set the [collision behaviour](https://frankarobotics.github.io/libfranka/0.18.0/classfranka_1_1Robot.html#a168e1214ac36d74ac64f894332b84534) according to the control law chosen,
- set the load on the robot.  

Eventually, we can print some useful debug information.  
In the same function i've defined the **robot_control_callback**. This is the callback we pass to the _libfranka_ control function that spawns a **real-time thread** and controls the robot at a fixed frequency of 1 kHz.  

### _robot_control_callback_  

The most important function to modify for the implementation of a different control strategy. Within the function:

- the robot status is **read**,
- the quantities useful for control are **computed**,
- the control input vector is **computed**,
- the portions of the status useful for **debugging** and **monitoring** purpose on the ROS2 network are copied into **dedicated structures**,
- the **internal status** of the robot and the **forward kinematics** are published.

The publication of the status and fkine of the robot in the same thread is done to always keep the updated state on the network. The _dedicated structures_ are simply **mutex guarded** structure used to copy the robot state and publish it from another thread, lightening the **main control thread**.  
The only thread that **needs** to be real-time is the control thread spawned by _libfranka_, so the other threads can be spawned with simple threads or can be used a much lower priority if spawned in real-time.  

An example of a robot_control_callback function can be found in the _impedance_controller_ node of the project of course. Furthermore, the function used in the impedance controller node implements a **torque level** control: the _libfranka_ library accepts also function that allows to control the robot at higher level, with joint or cartesian velocites signals for example.

## *on_activate*  

The second function that should be modified is the _on_activate_ one. This function, like _on_configure_, is divided in 2 sections: one is common to both the modes of operation and one is specialized depending on the mode chosen.  
If the robot is controlled in the simulated environment, the function simply spawns the control thread that communicates with the gazebo env through the publisher and the subscribers of the netowrk. The control thread function is named ```control()``` and is almost identical to the _robot_control_callback_, implementing the same control law.  
If the robot is controlled through the FCI the function spawns several threads, each with its own purpose: 

- a thread that publishes only the `Pose`s of the robot's frames.
- A thread that updates internal compliance mode variables, allowing the robot to compute jacobian, pose, and current external wrench, depending on the operator's point of contact with the robot, and modifying the controller gains on the fly. 
- A thread publishing debug infos, reading the **mutex guarded** structure shared with the _robot_control_callback_.
- The control thread spawned with the [**control**](https://frankarobotics.github.io/libfranka/0.18.0/classfranka_1_1Robot.html#a0d5effba5daff2fee123802bbd5f95d1) function of the _libfranka_ lib, after configuring the SCHED_FIFO priority to 99.  

The function can be easily expanded adding other threads one would like to spawn.

## Publishers, subscribers, services and parameters  

Of course, if one wants to implement different control laws, one would need additional publisher/subscriber objects to communicate with the ROS2 network, other services to implement different behaviors, or new parameters to be specified. All of this can be implemented using the controller class as a starting template, communicating with the FCI using the object's internal structures and implementing the necessary logic within callbacks, threads, and the _robot_control_callback_ function.
