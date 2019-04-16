ssh ubuntu@10.9.0.8 "source ~/2019RobotCode/zebROS_ws/ROSJetsonMaster.sh && roslaunch adi_driver adis16470.launch with_rviz:=true" &
sleep 5
$HOME/2019RobotCode/zebROS_ws/launch_ros.sh src/adi_driver/launch/imu.rviz &
