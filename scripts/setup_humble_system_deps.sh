#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-humble}"

if [ "${EUID}" -ne 0 ]; then
  echo "Run this once with sudo:"
  echo "  sudo $0"
  exit 1
fi

if [ "${ROS_DISTRO}" != "humble" ]; then
  echo "This setup script is for ROS 2 Humble. Current ROS_DISTRO=${ROS_DISTRO}"
  exit 1
fi

if [ -r /etc/os-release ]; then
  # shellcheck disable=SC1091
  . /etc/os-release
  if [ "${ID:-}" != "ubuntu" ] || [ "${VERSION_ID:-}" != "22.04" ]; then
    echo "Warning: expected Ubuntu 22.04, found ${PRETTY_NAME:-unknown}."
  fi
fi

if [ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
  echo "ROS 2 Humble was not found at /opt/ros/${ROS_DISTRO}."
  echo "Install ROS 2 Humble first, then rerun this script."
  exit 1
fi

WITH_FULL=0
WITH_MOLA=0
for arg in "$@"; do
  case "${arg}" in
    --full)
      WITH_FULL=1
      WITH_MOLA=1
      ;;
    --with-mola)
      WITH_MOLA=1
      ;;
    *)
      echo "Unknown option: ${arg}"
      echo "Usage: sudo $0 [--full] [--with-mola]"
      exit 1
      ;;
  esac
done

apt-get update
apt-get install -y --no-install-recommends \
  build-essential \
  cmake \
  curl \
  git \
  git-lfs \
  iproute2 \
  gstreamer1.0-libav \
  gstreamer1.0-plugins-bad \
  gstreamer1.0-plugins-base \
  gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-ugly \
  gstreamer1.0-tools \
  libassimp-dev \
  libboost-all-dev \
  libccd-dev \
  libconsole-bridge-dev \
  libeigen3-dev \
  libhidapi-dev \
  libhidapi-hidraw0 \
  libhidapi-libusb0 \
  liboctomap-dev \
  libpcl-dev \
  libtinyxml2-dev \
  libusb-1.0-0 \
  liburdfdom-dev \
  liburdfdom-headers-dev \
  libxcb-cursor0 \
  libxcb-icccm4 \
  libxcb-image0 \
  libxcb-keysyms1 \
  libxcb-render-util0 \
  libxcb-shape0 \
  libxcb-xinerama0 \
  libxcb-xinput0 \
  libxkbcommon-x11-0 \
  pkg-config \
  python3-catkin-pkg \
  python3-colcon-common-extensions \
  python3-empy \
  python3-evdev \
  python3-lark \
  python3-numpy \
  python3-pip \
  python3-rosdep \
  python3-scipy \
  python3-vcstool \
  python3-venv \
  python3-yaml \
  ros-${ROS_DISTRO}-ament-index-python \
  ros-${ROS_DISTRO}-cyclonedds \
  ros-${ROS_DISTRO}-interactive-markers \
  ros-${ROS_DISTRO}-joint-state-publisher-gui \
  ros-${ROS_DISTRO}-joy \
  ros-${ROS_DISTRO}-pcl-conversions \
  ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
  ros-${ROS_DISTRO}-robot-state-publisher \
  ros-${ROS_DISTRO}-rosidl-default-generators \
  ros-${ROS_DISTRO}-rviz2 \
  ros-${ROS_DISTRO}-tf-transformations \
  ros-${ROS_DISTRO}-xacro

if [ "${WITH_FULL}" -eq 1 ]; then
  apt-get install -y --no-install-recommends \
    doxygen \
    graphviz \
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    libqglviewer-dev-qt5 \
    libqt5opengl5-dev \
    libyaml-cpp-dev
fi

if [ "${WITH_MOLA}" -eq 1 ]; then
  apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-mola \
    ros-${ROS_DISTRO}-mola-lidar-odometry \
    ros-${ROS_DISTRO}-mola-state-estimation
fi

if command -v rosdep >/dev/null 2>&1; then
  rosdep init 2>/dev/null || true
  if [ -n "${SUDO_USER:-}" ] && [ "${SUDO_USER}" != "root" ]; then
    sudo -u "${SUDO_USER}" rosdep update || true
  else
    rosdep update || true
  fi
fi

echo
echo "System dependencies installed for Ubuntu 22.04 + ROS 2 Humble."
if [ "${WITH_FULL}" -eq 1 ]; then
  echo "Full native build apt dependencies were included."
fi
echo "For joystick access, add each lab user to the input group if needed:"
echo "  sudo usermod -aG input <username>"
echo "Users must log out and back in after group changes."
