# Autonomous-mobile-robot-for-industrial-workspaces

To launch using kinect only :


roslaunch freenect_launch freenect.launch depth_registration:=true

roslaunch rtabmap_ros rtabmap.launch rtabmap_args:="--delete_db_on_start"

roslaunch rtabmap_ros planner.launch
