## container 없이 실행
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
# fsm_waypoint 실행
export CONFIG_FILE=/home/bcc/Works1/rmf_demos_server/rmf_demos/config/office/tinyRobot_with_nav2_config.yaml && \
  colcon build --packages-select fsm_waypoint && \
  export ROS_DOMAIN_ID=3 && \
  ros2 run fsm_waypoint fsm_waypoint_node
```
## build
```bash
docker compose build iron
docker compose build rmf
```
## usage
```bash
xhost +
touch .env
다음과 같이 작성 123은 예시임
ROS_DOMAIN_ID=123
DISPLAY=0

docker compose up rmf api dashboard panel

# open browser
# http://localhost:3000/ <= dashboard
# http://localhost:3001/  <= panel
# http://localhost:8000/docs/  <= api document
```
- rviz_satellite 실행
1. ros2 launch rviz_satellite demo.launch.xml 실행전 demo.launch.xml에 위경도 입력
2. aerialmap_display.cpp에서 x_offset, y_offset 입력
3. gl.rviz 파일에 - /AerialMap1, Class: rviz_satellite/AerialMap 추가
4. 위경도, x,y_offset은 0.yaml를 보고 참조, 즉 traffic_editor에 vector의 위경도와 build후 생성된 0.yaml를 참조
5. rviz_satellite 실행

```bash
## etc
- 새로운 맵을 만드는 방법
  - traffic_editor로 vector, floor 등 만든다
  - rmf_demos_maps/maps/ 원하는 폴더를 만들고 *.building.yaml 저장위치
  - rmf_demos_dashboard_resources/ 원하는 폴더와 dashboard_config.json, main.json을 타 폴더내용 참고해서 작성할것 후에 rmf-panel-js에서 사용함
  - rmf_demos/config/ 원하는 폴더에 *_config.yaml 파일작성
  - rmf_demos/launch/ *.launch.xml 파일작성

## task
```bash
# tinybot1이 lounge와 coe를 순차적으로 patrol 이동
ros2 run rmf_demos_tasks dispatch_patrol -F turtlebot3 -R tinybot1 -p lounge coe

# tinybot1이 lounge와 coe를 순차적으로 patrol 2번 반복
ros2 run rmf_demos_tasks dispatch_patrol -F turtlebot3 -R tinybot1 -p lounge coe -n 2

# task 실행시 id를 사용해서 task 취소
ros2 run rmf_demos_tasks cancel_task -id patrol_45a22ccc-d625-445b-ab25-7c655aca5e02

# rviz_satellite 실행
ros2 launch rviz_satellite demo.launch.xml

# 취소
export ROS_DOMAIN_ID=13 &&
ros2 run rmf_demos_tasks cancel_task -id patrol_45a22ccc-d625-445b-ab25-7c655aca5e02

export ROS_DOMAIN_ID=3 && ros2 run rmf_demos_tasks dispatch_patrol -F sim_bot -R tinybot1 -p c0
```
