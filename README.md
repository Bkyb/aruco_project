# aruco_project
Aruco project and camera calibration, using doosan robot and realsense.

Actually, I didn't build this code. It's just a modification of the original code.



# How to use?

1. Connecting Doosan Robot
> roslaunch dsr_launcher single_robot_rviz.launch model:=m1013 mode:=real host:=192.168.137.100
2. Connecting Realsense (If you want, you can use customized launch file.)
> roslaunch realsense2_camera rs_aligned_depth.launch
3. RUN
> roslauch campus_pj campus_pj_demo.launch

