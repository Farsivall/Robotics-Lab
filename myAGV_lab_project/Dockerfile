# syntax=docker/dockerfile:1
# ──────────────────────────────────────────────────────────────────────────────
# myAGV Summer School Lab — Docker Image
# Elephant Robotics myAGV 2023
#
# Workspace layout mirrors the real robot container (myagv_slam):
#   /ws/src/myAGV_lab_project   ← project source
#   /ws/install/                ← colcon install space
#
# Supports two build targets:
#   sim   — Python + dependencies only, no ROS2 (student laptops)
#   robot — Full ROS2 environment (real myAGV 2023 board)
#
# The ROS2 distro is set via build-arg ROS_DISTRO.
# Default is "galactic" to match the existing robot container.
# Override for newer setups: --build-arg ROS_DISTRO=lyrical
#
# Build:
#   docker build --target sim   -t myagv-lab:sim   .
#   docker build --target robot -t myagv-lab:robot .
#
# Quick start (simulation):
#   docker compose run --rm sim
#
# Platform:
#   linux/amd64 (student laptops) and linux/arm64 (myAGV on-board).
#   Multi-arch: docker buildx build --platform linux/amd64,linux/arm64
# ──────────────────────────────────────────────────────────────────────────────

ARG ROS_DISTRO=galactic
ARG UBUNTU_VERSION=20.04

# ══════════════════════════════════════════════════════════════════════════════
#  BASE — shared Ubuntu layer (Python + display libs)
# ══════════════════════════════════════════════════════════════════════════════
FROM ubuntu:${UBUNTU_VERSION} AS base

ARG DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV PYTHONUNBUFFERED=1

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3 \
    python3-pip \
    python3-venv \
    python3-dev \
    # Matplotlib / X11 (software rendering, no GPU needed)
    libgl1-mesa-glx \
    libglib2.0-0 \
    libsm6 \
    libxext6 \
    libxrender1 \
    # Utilities
    curl \
    git \
    ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# Python dependencies — installed before copying source for better layer caching
WORKDIR /ws/src/myAGV_lab_project
COPY requirements.txt .
RUN pip3 install --no-cache-dir -r requirements.txt

# ══════════════════════════════════════════════════════════════════════════════
#  SIM TARGET — no ROS2, lightweight (student laptops)
# ══════════════════════════════════════════════════════════════════════════════
FROM base AS sim

ENV MYAGV_USE_SIM=1

COPY . /ws/src/myAGV_lab_project/
RUN pip3 install --no-cache-dir -e .

RUN useradd -m -s /bin/bash student && chown -R student:student /ws
USER student

WORKDIR /ws/src/myAGV_lab_project
CMD ["bash"]

# ══════════════════════════════════════════════════════════════════════════════
#  ROS2 BASE — ROS2 + Nav2 + SLAM Toolbox
# ══════════════════════════════════════════════════════════════════════════════
FROM base AS ros2-base

ARG ROS_DISTRO=galactic

# Add the official ROS2 apt repository
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) \
          signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
          http://packages.ros.org/ros2/ubuntu \
          $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
        > /etc/apt/sources.list.d/ros2.list

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-ros-base \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-slam-toolbox \
    ros-${ROS_DISTRO}-nav2-map-server \
    ros-${ROS_DISTRO}-rmw-fastrtps-cpp \
    python3-colcon-common-extensions \
    python3-rosdep \
    && rosdep init && rosdep update \
    && rm -rf /var/lib/apt/lists/*

ENV ROS_DISTRO=${ROS_DISTRO}

# Source ROS2 automatically in every interactive shell
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /etc/bash.bashrc && \
    echo "source /ws/install/setup.bash 2>/dev/null || true" >> /etc/bash.bashrc

# ══════════════════════════════════════════════════════════════════════════════
#  ROBOT TARGET — full ROS2 environment (real myAGV 2023)
# ══════════════════════════════════════════════════════════════════════════════
FROM ros2-base AS robot

ENV MYAGV_USE_SIM=0

COPY . /ws/src/myAGV_lab_project/
WORKDIR /ws/src/myAGV_lab_project
RUN pip3 install --no-cache-dir -r requirements.txt && \
    pip3 install --no-cache-dir -e .

# Build the ROS2 colcon workspace
RUN /bin/bash -c "\
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    cd /ws && \
    colcon build --symlink-install --packages-select myagv_lab"

RUN useradd -m -s /bin/bash student && chown -R student:student /ws
USER student

WORKDIR /ws/src/myAGV_lab_project

ENTRYPOINT ["/bin/bash", "-c", \
    "source /opt/ros/${ROS_DISTRO}/setup.bash && \
     source /ws/install/setup.bash 2>/dev/null || true && \
     exec \"$@\"", "--"]
CMD ["bash"]
