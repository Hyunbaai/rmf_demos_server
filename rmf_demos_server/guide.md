## container 없이 실행
```bash
# api-server 실행
docker run \
   --network host \
   -it --rm \
   -e ROS_DOMAIN_ID=13 \
   -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
   ghcr.io/open-rmf/rmf-web/api-server:latest
# dashboard frontend 실행
docker run \
   --network host -it --rm \
   -e RMF_SERVER_URL=http://localhost:8000 \
   -e TRAJECTORY_SERVER_URL=ws://localhost:8006 \
   ghcr.io/open-rmf/rmf-web/dashboard:latest

# rviz를 포함한 office 런치 파일 실행
export ROS_DOMAIN_ID=13 &&
ros2 launch rmf_demos xam.launch.xml \
  server_uri:="ws://localhost:8000/_internal" \
  use_sim_time:=False \
  headless:=False
```