FROM osrf/ros:noetic-desktop-full

RUN sudo apt-get update

RUN sudo apt-get install -y python3.8-venv \
        python3-pip \
        ros-noetic-joy \
        joystick

RUN pip3 install wheel

WORKDIR /app

COPY . /app

RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
