# 默认只在本机做 DDS 发现：实验室网段另一台机器也在发 /clock，不隔离的话
# 仿真会刷 "Detected jump back in time"。需要跨机通信时先 export ROS_LOCALHOST_ONLY=0 再 source 本文件。
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"

source /opt/ros/humble/setup.bash
source /home/lwt/wust_heavy_lift/modules/arm_planning/install/setup.bash
source /home/lwt/wust_heavy_lift/modules/vision/install/setup.bash
export PYTHONPATH=/home/lwt/wust_heavy_lift/modules/vision/perception:$PYTHONPATH
