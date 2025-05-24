[no-cd]
build:
	colcon build

[no-cd]
init: 
  build

[no-cd]
completion: 
	colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --cmake-clean-cache

[no-cd]
ros-pkg pkg-name:
  ros2 pkg create --build-type ament_cmake --license Apache-2.0 {{pkg-name}}

[no-cd]
build-pkg pkg-name:
	colcon build --packages-select {{pkg-name}}

