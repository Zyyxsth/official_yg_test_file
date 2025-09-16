# 输入sudo密码（⚠️请谨慎使用此方式保存密码）
PASSWORD=" "
# gnome-terminal -- bash -c "roscore; exec bash" & sleep 5;
gnome-terminal -- bash -c "cd dia_ws;source devel/setup.bash; roslaunch livox_ros_driver2 msg_MID360.launch; exec bash" & sleep 5;
gnome-terminal -- bash -c "cd dia_ws;source devel/setup.bash; roslaunch fastlio localize.launch; exec bash" & sleep 5;
gnome-terminal -- bash -c "cd Elastic-Tracker-main;source devel/setup.bash;roslaunch read_pcd read_real.launch; exec bash" & sleep 7;
gnome-terminal -- bash -c "cd dia_ws; source devel/setup.bash; rosservice call /slam_reloc '{pcd_path: \"/home/hialb/Elastic-Tracker-main/src/read_pcd/PCDFiles/outside_0305.pcd\", x: 0.0, y: 0.0, z: 0.0, roll: 0.0, pitch: 0.0, yaw: 0.0}'; exec bash" & sleep 5;
gnome-terminal -- bash -c "cd dia_ws;source devel/setup.bash; rosservice call /slam_reloc "{pcd_path: '/home/yg2/Elastic-Tracker-main/src/read_pcd/PCDFiles/outside_0305.pcd', x: 0.0, y: 0.0, z: 0.0, roll: 0.0, pitch: 0.0, yaw: 0.0}"; exec bash" & sleep 5;
# gnome-terminal -- bash -c "cd Elastic-Tracker-main;source devel/setup.bash;rosrun diablo_ctrl diablo_ctrl_node; exec bash" & sleep 5;
# gnome-terminal -- bash -c "cd Elastic-Tracker-main;source devel/setup.bash; sudo chmod 777 /dev/ttyUSB0; ;rosrun diablo_ctrl diablo_ctrl_node; exec bash" & sleep 5;
gnome-terminal -- bash -c "cd Elastic-Tracker-main;source devel/setup.bash;rosrun diablo_body testmpc.py; exec bash" & sleep 5;
gnome-terminal -- bash -c "cd Elastic-Tracker-main;source devel/setup.bash;rosrun diablo_body rotation.py; exec bash" & sleep 5;
gnome-terminal -- bash -c "cd Elastic-Tracker-main;source devel/setup.bash;roslaunch planning fake_target.launch; exec bash" & sleep 5;
gnome-terminal -- bash -c "cd Elastic-Tracker-main;cd sh_utils;./bag.sh; exec bash";
