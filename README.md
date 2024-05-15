# AIS2105_3DOF

# cam node
v4l2-ctl --list-devices
ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:=/dev/video0

# avalable nodes


# launch file
ros2 launch ais_3dof nodes.launch.py model:=/home/kristian/workspace/ros2_ws/src/ais_3dof/urdf/three_dof.urdf
