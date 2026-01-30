# Autonomous-vehicle


some links: 

https://dspace.mit.edu/bitstream/handle/1721.1/119149/16-412j-spring-2005/contents/projects/1aslam_blas_repo.pdf

https://lidarlist.com/

https://www.formulastudent.de/fileadmin/user_upload/all/2025/rules/FS-Rules_2025_v1.1.pdf

https://github.com/TUMFTM

https://github.com/changh95/visual-slam-roadmap?tab=readme-ov-file

BASIC ROBOT WITH CASTOR WHEEL GEOMETRY :
<img width="664" height="778" alt="image" src="https://github.com/user-attachments/assets/927d64a4-8099-4be4-9fe6-5dad6aad7f5f" />

always remember to bridge ros2 and gazebo with this command



ros2 run ros_gz_bridge parameter_bridge \
/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist


Adding any sensor

Make sure you have correct sdf format which can be checked by creating an empty world (with .sdf extension)and then opening it. What this does is that it helps in understanding how different sensors like a lidar,cameraetc have to integrated in the xml file for us 


First Lidar simulation

<img width="780" height="509" alt="image" src="https://github.com/user-attachments/assets/c9254025-af2b-4171-87cc-d64844977683" />
<img width="780" height="509" alt="image" src="https://github.com/user-attachments/assets/9fca9af0-8968-437b-843b-ba669d11e926" />

Always make sure your bridging parameters works well otherwise faults will keep on generating 


NEW DESIGN FOR THE ROBOT

S<img width="578" height="500" alt="image" src="https://github.com/user-attachments/assets/38b8704b-876e-479e-bc6e-6ce82d6cd461" />

Implementation of new design
<img width="975" height="439" alt="image" src="https://github.com/user-attachments/assets/5a2ee62a-d842-44ec-b9db-1db2aa5195fe" />

NAVIGATION IMPLEMENTATION


<img width="758" height="305" alt="image" src="https://github.com/user-attachments/assets/ecaf9916-6298-4f51-82b4-1a35e4c8ed36" />


converted an image file to yaml file and then uplaod it to rviz2


<img width="1202" height="804" alt="image" src="https://github.com/user-attachments/assets/7035b6e5-4d04-485d-ae07-32285bfe9c53" />

for generating the map 

cd ~/autonomous-vehicle-/src/mobile_robot
source install/setup.bash

ros2 run nav2_map_server map_server \
  --ros-args \
  -p yaml_filename:=maps/test_track.yaml



