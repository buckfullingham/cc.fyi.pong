FROM ubuntu

RUN apt -y update \
 && apt -y upgrade \
 && apt -y install \
    cmake \
    python3-full \
    libgl-dev \
    pkg-config \
    gdb \
    g++ \
    libglfw3-dev \
    xorg-dev

RUN python3 -m venv /venv
ENV PATH=$PATH:/venv/bin
COPY ../requirements.txt /tmp/
RUN pip3 install -U -r /tmp/requirements.txt && rm /tmp/requirements.txt

RUN apt -y install libx11-xcb-dev
RUN apt -y install libwlroots-dev
RUN apt -y install libxcb-glx0-dev
RUN apt -y install libxcb-cursor-dev
RUN apt -y install libxcb-util-dev
RUN apt -y install libxcb-imdkit-dev
RUN apt -y install libxcb-present-dev
RUN apt -y install libxcb-cursor-dev
RUN apt -y install libxkbcommon-x11-dev
RUN apt -y install libxcb-dri2-0-dev
RUN apt -y install libxcb-xinerama0-dev

ENV BUILD_PROFILE=linux
RUN useradd pong
USER pong
