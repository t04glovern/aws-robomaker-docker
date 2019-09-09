# Kudos to DOROWU for his amazing VNC 16.04 KDE image
FROM dorowu/ubuntu-desktop-lxde-vnc
LABEL maintainer "bpinaya@wpi.edu"

RUN apt-get update && apt-get install -y dirmngr

# Adding keys for ROS
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# Installing ROS
RUN apt-get update && apt-get install -y ros-melodic-desktop-full \
		wget git nano python-rosinstall python3-colcon-common-extensions
RUN rosdep init && rosdep update

RUN /bin/bash -c "echo 'export HOME=/home/ubuntu' >> /root/.bashrc && source /root/.bashrc"

# Creating catkin_ws
RUN mkdir -p /home/ubuntu/catkin_ws/src

# Set up the workspace
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash && \
                  cd /home/ubuntu/catkin_ws/ && \
                  catkin_make && \
                  echo 'source /home/ubuntu/catkin_ws/devel/setup.bash' >> ~/.bashrc"

# Installing modules
COPY . /home/ubuntu/catkin_ws/src/

# Updating ROSDEP and installing dependencies
RUN cd /home/ubuntu/catkin_ws && sudo rosdep fix-permissions && rosdep update && rosdep install --from-paths src --ignore-src --rosdistro=melodic -y

# Adding scripts and adding permissions
RUN cd /home/ubuntu/catkin_ws/src/scripts && \
		chmod +x build.sh && \
		chmod +x bundle.sh && \
		chmod +x setup.sh

# Sourcing
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash && \
                  cd /home/ubuntu/catkin_ws/ && rm -rf build devel && \
                  catkin_make"

# Dunno about this one tbh
RUN /bin/bash -c "echo 'source /home/ubuntu/catkin_ws/devel/setup.bash' >> /root/.bashrc"
