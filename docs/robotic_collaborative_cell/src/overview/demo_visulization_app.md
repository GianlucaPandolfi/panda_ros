# demo_visualization_app  

The *demo_visualization_app* package contains a single node, runned without the usage of launch files, that spawns a gui capable of running and stopping both the controller and the demo. Furthermore, the gui can be used to check the status of the demo and the frame in contact with the operator, depending on the color and the label in the central section of the window.  

## Nodes  

### *color_node_app*  

*color_node_app* is the only node in the package, it can be runned through the `ros2 run` command specifying the topic the app has to listen on for the state of the demo, and the topic where the wrist and the frame in contact are published. The name of the service and the bash script that the node execute are specified in the file.  
The **state topic** is expected to publish `ColorRGBA` messages indicating the state of the demo with a state associated to each color: 

- black: no_state
- task: green
- yellow: transition_human
- red: compliance
- blue: transition_leave_human

The other topic expects a `HumanContact` message containing the name of the wrist and frame in contact.  
The buttons of the app are used to:
- activate/deactivate the impedance controller: through 2 different bash scripts that activate/deactivate the impedance lifecycle node.  
- run/stop the demo: using the service exposed by the demo node. 

An example of the ROS2 command to spawn the gui with the predefined topic names and the bash script in the repo is: 

```bash
ros2 run demo_visualization_app color_app "state_color" "human_contact"
```
