FROM osrf/ros:noetic-desktop-buster

WORKDIR /root/src
COPY src/ .
RUN find . \! -type f -path "./*/package.xml" -print0 | xargs -0 rm -df

FROM osrf/ros:noetic-desktop-buster

RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential python3-rosdep python3-rosinstall python3-vcstools \
    mesa-utils mesa-utils-extra libxv1 va-driver-all xwayland \
    libgl1-mesa-glx libgl1-mesa-dri

WORKDIR /root
VOLUME /root
COPY --from=0 /root .

RUN rosdep install --from-paths src --ignore-src -y \
    && rm -rf /var/lib/apt/lists/*
