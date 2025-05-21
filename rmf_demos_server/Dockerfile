FROM rmf_iron
ARG DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

## 추가 의존성 설치
RUN apt-get update && apt-get install -y \
    ros-$ROS_DISTRO-rviz2  \
    ros-$ROS_DISTRO-xacro \
    ros-$ROS_DISTRO-twist-mux \
    ros-$ROS_DISTRO-interactive-marker-twist-server \
    ros-$ROS_DISTRO-rqt ros-$ROS_DISTRO-rqt-graph \
    ros-$ROS_DISTRO-rmf-dev \
    ros-$ROS_DISTRO-tf-transformations

## install smach
RUN apt-get update && \
    apt-get install -y python3-pip && \
    pip install --upgrade pip &&  \
    pip install requests psutil py_trees pytz geographiclib supabase \
    fastapi \
    uvicorn \
    websocket-client \
    flask-socketio \
    flask-cors \
    websockets

WORKDIR /root/3rdparty
RUN git clone https://github.com/gabrielsr/standalone-smach.git standalone-smach && \
    sed -i '247s/isAlive()/is_alive()/g' standalone-smach/smach/concurrence.py && \
    cd standalone-smach && \
    python3 setup.py install

WORKDIR /root/ws/src
COPY rmf_demos rmf_demos
COPY rmf_demos_assets rmf_demos_assets
COPY rmf_demos_bridges rmf_demos_bridges
COPY rmf_demos_dashboard_resources rmf_demos_dashboard_resources
COPY rmf_demos_fleet_adapter rmf_demos_fleet_adapter
COPY rmf_demos_maps rmf_demos_maps
COPY rmf_demos_panel rmf_demos_panel
COPY rmf_demos_tasks rmf_demos_tasks
COPY rviz_satellite rviz_satellite

WORKDIR /root/ws
RUN apt-get update && source /opt/ros/$ROS_DISTRO/setup.sh && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build

RUN echo 'alias ls="ls --color=auto"' >> ~/.bashrc \
    && echo "source /root/ws/install/setup.bash" >> /root/.bashrc
