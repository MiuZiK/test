#! /bin/bash 

gnome-terminal -- bash -c "
  source /opt/ros/noetic/setup.bash;
  cd ~/lan_planner/start_scripts/;
  ./odom.sh;
  exec bash" &
sleep 5 


gnome-terminal -- bash -c "
  source /opt/ros/noetic/setup.bash;
  cd ~/lan_planner/start_scripts/;
  ./run_real.sh;
  exec bash" &
sleep 2

gnome-terminal -- bash -c "
  source /opt/ros/noetic/setup.bash;
  cd ~/lan_planner/;
  source devel/setup.bash;
  roslaunch offboard offboard.launch;
  exec bash" &
sleep 2
  
gnome-terminal -- bash -c "
  cd ~/mediamtx/;
  ./start_stream.sh;
  exec bash" &
sleep 5



#gnome-terminal -- bash -c "
#  source /opt/ros/noetic/setup.bash;
 # cd ~/lan_planner/;
 # source devel/setup.bash;
 # rosrun offboard click_points.py
 # exec bash" &
sleep 5

#gnome-terminal -- bash -c "
 # source /opt/ros/noetic/setup.bash;
 # cd ~/lan_planner/;
 # source devel/setup.bash;
 # rosrun offboard odometry_detection.py
 # exec bash" &
sleep 2
 
gnome-terminal -- bash -c "
  source /opt/ros/noetic/setup.bash;
  cd ~/lan_planner/start_scripts/bag;
  rosbag record /livox/imu /livox/lidar -O lastest.bag
 # exec bash"
  

