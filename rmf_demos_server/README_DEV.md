
## fsm_waypoint 이름과 다른 기능 실행 방법
```bash
# export CONFIG_FILE=/home/bcc/Works1/rmf_demos_server/rmf_demos/config/office/tinyRobot_with_nav2_config.yaml && \
# export CONFIG_FILE=/home/bcc/Works1/rmf_demos_server/rmf_demos/config/xam/xam_with_nav2_config.yaml && \
# export CONFIG_FILE=/home/zetabank/ros2_ws/src/rmf_server/rmf_demos/config/handong/handong_deliveryRobot_config.yaml  && \
export CONFIG_FILE=/home/bcc/Works1/rmf_demos_server/rmf_demos/config/xam/xam_with_nav2_config.yaml && \
  colcon build --packages-select fsm_waypoint && \
  export ROS_DOMAIN_ID=3 && \
  ros2 run fsm_waypoint fsm_waypoint_node
  

# GIS
ros2 launch rviz_satellite demo.launch.xml
```
## traffic editor tip
- bidirectional default true, false로 변경하면 rviz에서 선 굵기는 같고 투명도가 다름
- speed_limit default 0, 0이상이면 rviz에서 선 굵기가 더 얇음, /robot_state에서 approach_speed_limit이 적용됨
  - 속도적용은 core가 전달해준 path의 시작과 끝사이에 speed_limit가 >0이면 표시됨,
  - 예로 a--b--c 이렇게 path가 있을때 b--c에 적용된 10값이 있을때 core가 b--c path으면 적용, a--c이면 미적용됨
## responsive_wait
- robot마다 적용할수 있음
- a, b로봇의 true이고 a가 갈 지점에 b가 있다면 b가 회피위치로 이동후 a가 지나가면 다시 원위치로 옮! 오~
- 
## 작업 환경
- Ubuntu 22.04
- ROS2 iron
- rmf_demo 2.2.3


## 실행 방법
```bash
# api-server 실행
docker run \
   --network host \
   -it --rm \
   -e ROS_DOMAIN_ID=3 \
   -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
   ghcr.io/open-rmf/rmf-web/api-server:latest
# dashboard frontend 실행
docker run \
   --network host -it --rm \
   -e RMF_SERVER_URL=http://localhost:8000 \
   -e TRAJECTORY_SERVER_URL=ws://localhost:8006 \
   ghcr.io/open-rmf/rmf-web/dashboard:latest

# rviz를 포함한 office 런치 파일 실행
export ROS_DOMAIN_ID=3 &&
ros2 launch rmf_demos office.launch.xml \
  server_uri:="ws://localhost:8000/_internal" \
  use_sim_time:=False \
  headless:=False
```

## build 방법
```bash
sudo apt-get install ros-$ROS_DISTRO-tf-transformations
pip3 install fastapi uvicorn websocket-client
python3 -m pip install flask-socketio
sudo apt update && sudo apt install ros-$ROS_DISTRO-rmf-dev
```


## trajectory 죽었을때
```bash
export ROS_DOMAIN_ID=3 &&
ros2 run rmf_visualization_schedule schedule_visualizer_node \
    --ros-args \
    -p use_sim_time:=false \
    -p rate:=10.0 \
    -p path_width:=0.2 \
    -p initial_map_name:=L1 \
    -p wait_secs:=10 \
    -p port:=8006 \
    -p retained_history_count:=50
    
```

## trajectory 확인
- export ROS_DOMAIN_ID=3 && ros2 topic info /schedule_markers -v 확인하면 publisher count가 0이다.



## port 확인
- 8083 7878
- sudo netstat -tuln | grep 8083


## git clone 

- git checkout tags/2.2.3 -b branch-2.2.3


## task 실행
```bash

  
export ROS_DOMAIN_ID=3 &&
ros2 run rmf_demos_tasks dispatch_patrol -p north_east south_east -n 3

export ROS_DOMAIN_ID=3 &&
ros2 run rmf_demos_tasks dispatch_patrol -p turtlebot3_1_charger north_east south_east north_west turtlebot3_1_charger south_west -n 10


export ROS_DOMAIN_ID=3 && 
ros2 run rmf_demos_tasks dispatch_patrol -p coe pantry lounge supplies hardware_2 -n 10
patrol_D2
patrol_A1
patrol_D1
pantry
lounge
hardware_2
coe
tinyRobot3_charger
patrol_C
patrol_B
supplies
patrol_A2
export ROS_DOMAIN_ID=3 && 
ros2 run rmf_demos_tasks dispatch_patrol -F turtlebot3 -R tinybot1 -a patrol_A1 -s coe -n 4

export ROS_DOMAIN_ID=3 && 
ros2 run rmf_demos_tasks dispatch_patrol -F turtlebot3 -R tinybot1 -p pantry coe hardware_2 -n 4
export ROS_DOMAIN_ID=3 && 
ros2 run rmf_demos_tasks dispatch_patrol -F turtlebot3 -R tinybot2 -p coe pantry lounge -n 4  
export ROS_DOMAIN_ID=3 && 
ros2 run rmf_demos_tasks dispatch_patrol -F turtlebot3 -R tinybot3 -p patrol_B patrol_A2 patrol_C -n 4  

# target position 연속 실행
export ROS_DOMAIN_ID=3 && 
ros2 run rmf_demos_tasks patrol_manager 


export ROS_DOMAIN_ID=13 && 
ros2 run rmf_demos_tasks dispatch_patrol -F DeliveryRobot -R tank_orange -p target_01 target_02 target_03 charger_01 -n 10

export ROS_DOMAIN_ID=13 && 
ros2 run rmf_demos_tasks dispatch_patrol -F DeliveryRobot -R tank_orange -p tmp_03

export ROS_DOMAIN_ID=13 && 
ros2 run rmf_demos_tasks dispatch_patrol -F DeliveryRobot -R steer_yellow -p tmp_03

export ROS_DOMAIN_ID=13 && 
  ros2 run rmf_demos_tasks dispatch_action -F DeliveryRobot -R steer_yellow -a teleop -s tmp_03
export ROS_DOMAIN_ID=13 && 
  ros2 topic pub /action_execution_notice rmf_fleet_msgs/msg/ModeRequest '{fleet_name:  DeliveryRobot, robot_name: steer_yellow, mode: {mode: 0}}' --once

export ROS_DOMAIN_ID=13 && 
  ros2 run rmf_demos_tasks dispatch_action -F DeliveryRobot -R tank_orange -a teleop -s tmp_03
export ROS_DOMAIN_ID=13 && 
  ros2 topic pub /action_execution_notice rmf_fleet_msgs/msg/ModeRequest '{fleet_name:  DeliveryRobot, robot_name: tank_orange, mode: {mode: 0}}' --once


```
- turtlebot3, nav2, dispatch_action test
- 
```bash
export ROS_DOMAIN_ID=3 && ros2 run rmf_demos_tasks dispatch_action -F turtlebot3 -R tinybot3 -a teleop -s patrol_D1
export ROS_DOMAIN_ID=3 && ros2 topic pub /action_execution_notice rmf_fleet_msgs/msg/ModeRequest '{fleet_name:  turtlebot3, robot_name: tinybot3, mode: {mode: 0}}' --once

# 목적지로 이동
export ROS_DOMAIN_ID=3 && ros2 run rmf_demos_tasks dispatch_patrol -F turtlebot3 -R tinybot3 -p patrol_D1  
# 문닫힘 이벤트 발생
# 새로운 목적지로 이동
export ROS_DOMAIN_ID=3 && ros2 run rmf_demos_tasks dispatch_patrol -F turtlebot3 -R tinybot3 -p patrol_C 


```
