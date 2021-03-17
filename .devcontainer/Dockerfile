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
  && rm -rf /var/lib/apt/lists/*

ENV AMENT_PREFIX_PATH=/opt/ros/${ROS_DISTRO} \
    COLCON_PREFIX_PATH=/opt/ros/${ROS_DISTRO} \
    LD_LIBRARY_PATH=/opt/ros/${ROS_DISTRO}/lib:$LD_LIBRARY_PATH \
    PATH=/opt/ros/${ROS_DISTRO}/bin:$PATH \
    PYTHONPATH=/opt/ros/${ROS_DISTRO}/lib/python3.8/site-packages \
    # PYTHONPATH=/opt/ros/${ROS_DISTRO}/lib/python3.8/site-packages:$PYTHONPATH \
    ROS_PYTHON_VERSION=3 \
    ROS_VERSION=2 \
    DIRPATH=/usr/local/src \
    DEBIAN_FRONTEND=noninteractive

RUN apt-get upgrade && apt-get update && apt-get install -y \
    bash-completion \
    xvfb \
    unzip \
    ssh \
    gdb \
    curl \
    cmake \
    apt-utils \
    pkg-config \
    libjpeg-dev \
    libpng-dev \
    libtiff-dev \
    libvtk6-dev \
    libgtk-3-dev \
    libatlas-base-dev \
    software-properties-common \
    gfortran \
    pylint3 \
    python3-argcomplete \
    python3-colcon-common-extensions \
    python3-pip \
    # python3-autopep8 \
    && rm -rf /var/lib/apt/lists/*

# Add credentials on build and 
# Make sure your domain is accepted
ARG SSH_PRIVATE_KEY
RUN mkdir /root/.ssh/ && \
    echo "${SSH_PRIVATE_KEY}" > /root/.ssh/id_rsa && \
    chmod 600 ~/.ssh/id_rsa && \
    touch /root/.ssh/known_hosts && \
    ssh-keyscan -t rsa github.com >> /root/.ssh/known_hosts

# Install glog, gflags
RUN apt-get update && apt-get install -y \
    libgflags2.2 \
    libgflags-dev \
    libgoogle-glog0v5 \
    libgoogle-glog-dev

# Get python3
# TODO: Is this necessary?
RUN apt-get update && add-apt-repository ppa:deadsnakes/ppa
RUN apt-get update && apt-get install -y \
    python3.5 \
    python3.5-dev \
    python-pip \
    python3-pip \
    python-tk \
    python3-tk
RUN python3.5 -m pip install PyQt5==5.14

# Set the working directory and clone repos
WORKDIR $DIRPATH
COPY kimera.repos .
RUN vcs import < kimera.repos

# Build & Intsall OpenCV
RUN mkdir -p opencv/build && cd opencv/build && \
    cmake -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/usr/local \
    -D BUILD_opencv_python=OFF \
    -D BUILD_opencv_python2=OFF \
    -D BUILD_opencv_python3=OFF \
    -DOPENCV_EXTRA_MODULES_PATH=$DIRPATH/opencv_contrib/modules .. && \
    make -j $(nproc) install

# Build & Intsall GTSAM
RUN apt-get update && apt-get install -y libboost-all-dev libtbb-dev
RUN mkdir -p gtsam/build && cd gtsam/build && \
    cmake   -DCMAKE_INSTALL_PREFIX=/usr/local \
            -DGTSAM_USE_SYSTEM_EIGEN=OFF \
            -DGTSAM_BUILD_TESTS=OFF \
            -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
            -DCMAKE_BUILD_TYPE=Release \
            -DGTSAM_BUILD_UNSTABLE=ON \
            -DGTSAM_POSE3_EXPMAP=ON \
            -DGTSAM_ROT3_EXPMAP=ON \
            -DGTSAM_TANGENT_PREINTEGRATION=OFF .. && \
    make -j $(nproc) install

# Build & Intsall OpenGV
RUN mkdir -p opengv/build && cd opengv/build && \
    cmake -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/usr/local \
    -DEIGEN_INCLUDE_DIRS=$DIRPATH/gtsam/gtsam/3rdparty/Eigen \
    -DEIGEN_INCLUDE_DIR=$DIRPATH/gtsam/gtsam/3rdparty/Eigen .. && \
    make -j $(nproc) install

# Build & Intsall DBoW2
RUN mkdir -p DBoW2/build && cd DBoW2/build && \
    cmake .. && \
    make -j $(nproc) install

# Build & Intsall Kimera-RPGO
RUN mkdir -p Kimera-RPGO/build && cd Kimera-RPGO/build && \
    cmake .. && \
    make -j $(nproc) install

# Build & Install Kimera-VIO
RUN mkdir -p Kimera-VIO/build && cd Kimera-VIO/build && \
    sed -i '162,165 s/^/#/' ../CMakeLists.txt && \
    cmake -DKIMERA_BUILD_TESTS=OFF -DCMAKE_CXX_STANDARD=11 .. && \
    make -j $(nproc) install 

# Add to bash profile
RUN echo \
"source /usr/share/bash-completion/completions/git \n\
source /opt/ros/${ROS_DISTRO}/setup.bash \n\
source /usr/share/colcon_cd/function/colcon_cd.sh \n\
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash"\
>> /home/$USERNAME/.bashrc

# Add colcon underlay and any required dependencies
RUN mkdir -p /underlay/src 
WORKDIR /underlay
COPY underlay.* ./
RUN /bin/bash underlay.sh

ENV DEBIAN_FRONTEND=
COPY ros_entrypoint.sh /
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
USER ros