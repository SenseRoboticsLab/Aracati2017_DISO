FROM osrf/ros:noetic-desktop-full

# RUN 
RUN apt update && apt install vim -y
SHELL ["/bin/bash", "-c"]

# setup clion remote development
RUN apt-get update \
  && apt-get install -y ssh \
      build-essential \
      gcc \
      g++ \
      gdb \
      clang \
      make \
      ninja-build \
      cmake \
      autoconf \
      automake \
      locales-all \
      dos2unix \
      rsync \
      tar \
      python \
      git \
      sudo \
  && apt-get clean

RUN useradd -m da \
  && yes 123 | passwd da

RUN usermod -s /bin/bash da

# # Create the user 'da' and set password '123'
# RUN useradd -m -s /bin/bash da && echo 'da:123' | chpasswd

# Give 'da' sudo privileges
RUN usermod -aG sudo da

# To avoid password prompts for sudo commands, you can set 'da' to have passwordless sudo:
RUN echo "da ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

USER da
WORKDIR /home/da/diso_ws/src
RUN git clone https://github.com/SenseRoboticsLab/Aracati2017_DISO.git
WORKDIR /home/da/diso_ws
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && rosdep update && rosdep install --from-paths src --ignore-src -r -y"
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"
#
RUN echo "source /opt/ros/noetic/setup.bash" >> /home/da/.bashrc
RUN echo "source /home/da/diso_ws/devel/setup.bash" >> /home/da/.bashrc

CMD ["/bin/bash"]