## Dependencies  

```
sudo apt install ros-jazzy-{depth-image-proc,pinocchio,camera-info-manager,realtime-tools}
cd src/yolo_ros
pip install -r requirements.txt
```

## Configuration

```
export CYCLONEDDS_URI=/home/snasn/data/cyclonedds.xml
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```
