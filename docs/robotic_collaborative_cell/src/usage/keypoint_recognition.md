# Keypoint recognition  

The keypoint recognition routine, as described also in the [*image_processing* package](../overview/image_processing.md) section, is composed of a YOLO model returning a raw measure of the keypoints, then filtered from the only node of the package. The node can be modified to change the behaviour of the kalman filter, or the heuristic that tracks the skeleton in the image. Can be also included more than 1 skeleton to track inside the scene, based on the tracking done by the YOLO model. 

## _synchronized_callback_  

The *synchronized_callback* function contains all the high level logic of the node. As stated even in the overview of the package, the function gets the keypoint from the YOLO package message, process them and publish the results in the form of Tfs and `MarkerArray`. 

The `switch` on the state of the tracking can be modified to:
- implement a new state,
- change the [decision metrics](../overview/image_processing.md#decision-metrics),
- change the filter behaviour in the various states.  

The kalman filter can be modified externally, changing the measurement noise variance, or internally: 
- changing the linear system used for the estimation,
- including new outlier detection techniques.

Finally, one can include other measurements from different cameras, implementing a new piece of the pipeline, merging the measurements from the different cameras to obtain a more robust measurement of the operator's keypoints pose.
