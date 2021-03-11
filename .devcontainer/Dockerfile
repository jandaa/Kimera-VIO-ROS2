FROM ros:eloquent

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create a non-root user
RUN groupadd --gid $USER_GID $USERNAME \
  && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
  && apt-get update \
  && apt-get install -y sudo \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
  && chmod 0440 /etc/sudoers.d/$USERNAME \
  && rm -rf /var/lib/apt/lists/* \
  && echo "source /usr/share/bash-completion/completions/git" >> /home/$USERNAME/.bashrc \
  && echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> /home/$USERNAME/.bashrc

ENV AMENT_PREFIX_PATH=/opt/ros/${ROS_DISTRO} \
    COLCON_PREFIX_PATH=/opt/ros/${ROS_DISTRO} \
    LD_LIBRARY_PATH=/opt/ros/${ROS_DISTRO}/lib \
    PATH=/opt/ros/${ROS_DISTRO}/bin:$PATH \
    PYTHONPATH=/opt/ros/${ROS_DISTRO}/lib/python3.8/site-packages \
    ROS_PYTHON_VERSION=3 \
    ROS_VERSION=2 \
    DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    bash-completion \
    build-essential \
    gdb \
    git \
    curl \
    pylint3 \
    python3-argcomplete \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep \
    python3-vcstool \
    python3-autopep8 \
    && rm -rf /var/lib/apt/lists/* \
    && rosdep init || echo "rosdep already initialized"

### Taken from Kimera-VIO docker container ###

# Set the working directory to /root
RUN mkdir /ws && chmod 777 /ws
ENV DIRPATH /ws/
WORKDIR $DIRPATH

#Install build dependencies
RUN apt-get update && apt-get install -y --no-install-recommends apt-utils
RUN apt-get update && apt-get install -y git cmake build-essential pkg-config

# Install xvfb to provide a display to container for GUI realted testing.
RUN apt-get update && apt-get install -y xvfb

# Install OpenCV for Ubuntu 18.04
RUN apt-get update && apt-get install -y \
    unzip \
    libjpeg-dev libpng-dev libtiff-dev \
    libvtk6-dev \
    libgtk-3-dev \
    libatlas-base-dev gfortran

RUN git clone https://github.com/opencv/opencv.git
RUN cd opencv && \
    git checkout tags/3.3.1 && \
    mkdir build

RUN git clone https://github.com/opencv/opencv_contrib.git
RUN cd opencv_contrib && \
    git checkout tags/3.3.1
    
RUN cd opencv/build && \
    cmake -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/usr/local \
    -D BUILD_opencv_python=OFF \
    -D BUILD_opencv_python2=OFF \
    -D BUILD_opencv_python3=OFF \
    -DOPENCV_EXTRA_MODULES_PATH=$DIRPATH/opencv_contrib/modules .. && \
    make -j$(nproc) install

# Install GTSAM
RUN apt-get update && apt-get install -y libboost-all-dev libtbb-dev
ADD https://api.github.com/repos/borglab/gtsam/git/refs/heads/master version.json
RUN git clone https://github.com/borglab/gtsam.git
RUN cd gtsam && \
    git checkout ee069286b447ff58b809423cc77c777a02abdfe5 && \
    git fetch && \
    mkdir build && \
    cd build && \
    cmake   -DCMAKE_INSTALL_PREFIX=/usr/local \
            -DGTSAM_USE_SYSTEM_EIGEN=OFF \
            -DGTSAM_BUILD_TESTS=OFF \
            -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
            -DCMAKE_BUILD_TYPE=Release \
            -DGTSAM_BUILD_UNSTABLE=ON \
            -DGTSAM_POSE3_EXPMAP=ON \
            -DGTSAM_ROT3_EXPMAP=ON \
            -DGTSAM_TANGENT_PREINTEGRATION=OFF .. && \
    make -j$(nproc) install

# Install Open_GV
RUN git clone https://github.com/laurentkneip/opengv
RUN cd opengv && mkdir build
RUN cd opengv/build && \
    cmake -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/usr/local \
    -DEIGEN_INCLUDE_DIRS=$DIRPATH/gtsam/gtsam/3rdparty/Eigen \
    -DEIGEN_INCLUDE_DIR=$DIRPATH/gtsam/gtsam/3rdparty/Eigen .. && \
    make -j$(nproc) install

# Install DBoW2
RUN git clone https://github.com/dorian3d/DBoW2.git
RUN cd DBoW2 && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make -j $(nproc) install

# Install RobustPGO
ADD https://api.github.com/repos/MIT-SPARK/Kimera-RPGO/git/refs/heads/master version.json
RUN git clone https://github.com/MIT-SPARK/Kimera-RPGO.git
RUN cd Kimera-RPGO && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make -j $(nproc) install

RUN apt-get update && \
    apt-get install software-properties-common -y

# Get python3
RUN apt-get update && \
    add-apt-repository ppa:deadsnakes/ppa
RUN apt-get update && \
    apt-get install -y python3.5 python3.5-dev python-pip python3-pip python-tk python3-tk
RUN python3.5 -m pip install PyQt5==5.14

# Install evo-1 for evaluation
# Hack to avoid Docker's cache when evo-1 master branch is updated.
ADD https://api.github.com/repos/ToniRV/evo-1/git/refs/heads/master version.json
RUN git clone https://github.com/ToniRV/evo-1.git
RUN cd evo-1 && python3.5 $(which pip3) install .

# Install spark_vio_evaluation
RUN python3.5 $(which pip3) install ipython prompt_toolkit
# Hack to avoid Docker's cache when spark_vio_evaluation master branch is updated.
ADD https://api.github.com/repos/ToniRV/Kimera-VIO-Evaluation/git/refs/heads/master version.json
RUN git clone https://github.com/ToniRV/Kimera-VIO-Evaluation.git
# We use `pip3 install -e .` so that Jinja2 has access to the webiste template...
RUN cd Kimera-VIO-Evaluation && python3.5 $(which pip3) install -e .

# Add credentials on build
ARG SSH_PRIVATE_KEY
RUN mkdir /root/.ssh/ && sudo apt-get install -y ssh
RUN echo "${SSH_PRIVATE_KEY}" > /root/.ssh/id_rsa
RUN chmod 600 ~/.ssh/id_rsa

# Make sure your domain is accepted
RUN touch /root/.ssh/known_hosts
RUN ssh-keyscan -t rsa github.com >> /root/.ssh/known_hosts

# Install glog, gflags
RUN apt-get update && apt-get install -y libgflags2.2 libgflags-dev libgoogle-glog0v5 libgoogle-glog-dev

# Install Kimera-VIO
RUN git clone https://github.com/MIT-SPARK/Kimera-VIO.git
RUN cd Kimera-VIO && mkdir build && cd build && cmake .. && make -j$(nproc) install

# Install cv-bridge
RUN apt-get update && apt-get install -y \
    # ros-${ROS_DISTRO}-cv-bridge \
    gdb

# USER ros

# Build cv-bridge
RUN git clone https://github.com/ros-perception/vision_opencv.git && \
    cd vision_opencv && \
    git checkout ros2

# WORKDIR /ws/vision_opencv
# RUN /bin/bash -c "source /home/ros/.bashrc" && colcon build --symlink-install --packages-select cv_bridge

ENV DEBIAN_FRONTEND=

# WORKDIR /
# RUN mkdir -p /ws/repos/src
# COPY ros_entrypoint.sh /
# COPY kimera.repos /ws/repos/
# RUN . /opt/ros/eloquent/setup.sh && \
#     cd /ws/repos && \
#     vcs import src < kimera.repos && \
#     colcon build --merge-install

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

# # RUN mkdir -p /kimera
# # WORKDIR /kimera
# USER ros
# CMD /bin/bash while :; do sleep 100; done