FROM ros:humble-ros-core
ARG USERNAME=ros2user
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create the user
RUN groupadd -f --gid $USER_GID $USERNAME
RUN id -u $USERNAME >/dev/null 2>&1 || useradd --uid $USER_UID --gid $USER_GID -m $USERNAME

# Add sudo support
RUN apt-get -y update
RUN apt-get install -y sudo
RUN echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME
RUN chmod 0440 /etc/sudoers.d/$USERNAME
RUN apt-get update && apt-get upgrade -y

# Install a few important dependencies
RUN apt-get install -y ament-cmake
RUN apt-get install -y ccls
RUN apt-get install -y python3-colcon-common-extensions
RUN apt-get install -y python3-pip
RUN apt-get install -y vim
RUN apt-get install -y clang
RUN apt-get install -y clang-format
RUN apt-get install -y clang-tidy
RUN apt-get install -y ros-humble-rqt*
RUN apt-get install -y nano
RUN apt-get install -y screen
RUN apt-get install -y virtualenv

# Create workspace
RUN mkdir /home/ws

# Create virtual environment
RUN virtualenv /home/ws/.venv

# Set default shell
ENV SHELL=/bin/bash

CMD ["/bin/bash"]
