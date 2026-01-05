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
