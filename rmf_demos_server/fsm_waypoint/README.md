## 살행
```bash
vi handong_deliveryRobot_config.yaml

rmf_fleet:
  name: "handong_sim_fleet"   <= Partition key 이므로 잘 기입해야함, rmf_robot에서도 같은 이름으로 해야함!!!
  limits:
  ...
  robots:
    pink:
      charger: "Genesis_Station(pink)"
      responsive_wait: True
    ...
  zero_path: "/home/bcc/Works1/ws_server/install/rmf_demos_maps/share/rmf_demos_maps/maps/handong/nav_graphs/0.yaml" <= spwan_robot_name 기입 안해도됨


 export CONFIG_FILE=/home/zetabank/ros2_ws/src/rmf_server/rmf_demos/config/handong/handong_deliveryRobot_config.yaml  && \
  colcon build --packages-select fsm_waypoint && \
  export ROS_DOMAIN_ID=3 && \
  ros2 run fsm_waypoint fsm_waypoint_node
```

## voice
- aws polly
```ssml
<speak>
    o
    취소합니다.     
</speak>
```
## build
```bash

sudo apt install python3-pip
sudo apt update && sudo apt install ros-$ROS_DISTRO-rmf-dev
python3 -m pip install flask-socketio
pip3 install fastapi uvicorn websocket-client

mkdir -p ~/ws/3rdparty
cd ~/ws/3rdparty/
python3 --version

# install standalone-smach
git clone https://github.com/gabrielsr/standalone-smach.git standalone-smach && 
    sed -i '247s/isAlive()/is_alive()/g' standalone-smach/smach/concurrence.py &&  
    cd standalone-smach &&  
    sudo python3 setup.py install
    
# install tf-transformations
sudo apt-get install ros-$ROS_DISTRO-tf-transformations
sudo apt install ros-$ROS_DISTRO-gazebo-ros-pkgs

```
## usage
```bash
export ROS_DOMAIN_ID=6 &&
ros2 run fsm_waypoint fsm_waypoint_node
```
