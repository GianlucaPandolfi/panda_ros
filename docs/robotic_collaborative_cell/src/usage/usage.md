# Using the system  

The project contains a few files useful for the usage of the various components of the system. In addition, the nodes defined in the various packages can be easily modified to: 

- implement various controllers
- implement a different trajectory generator
- implement a different heuristic for the keypoint tracking
- using a different model to estimate the keypoints on the images
- using a different camera

## Bash scripts and lifecycle nodes  

The controller node is implemented as a [**lifecycle node**](https://docs.ros.org/en/jazzy/Tutorials/Demos/Managed-Nodes.html). To configure and activate the node some bash scripts are used, allowing to start/stop the controller, start both the inverse dyn and the impedance controller (for simulation), only activate the impedance controller. The scripts are located in the root of the repo.  
