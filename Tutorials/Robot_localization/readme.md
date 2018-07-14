# This is the directory for `robot_localization` tutorials
# NOTE: When launching IMUs; use launch files razor-pub0 and razor-pub1 for imu's 1 and 2 respectively. These files remap the imu's data into topisx /imu0 and /imu1 for use in fusing imu data. The change is only nescessary for launch files from 2018.07.13 and on. 
# NOTE:Launch file 2018.07.12.01_efk uses imu1 and imu2 that is an error i left in as my first attempt.
# NOTE:readQuatROS.m is used to convert the POSE estimates from Quaternions to Degrees.
# NOTE: Launch file 2018.07.13.01 and greater is meant to only take in 1 IMU's data until we can successfully parse the correct POSE msgs. Next is fusing to IMUs. Finally inccluding GPS data.
