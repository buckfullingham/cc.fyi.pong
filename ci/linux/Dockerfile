FROM ubuntu

RUN apt -y update && apt -y upgrade

RUN apt -y install \
    clang \
    cmake \
    python3-full \
    libgl-dev \
    pkg-config \
    gdb

RUN python3 -m venv /venv
ENV PATH=$PATH:/venv/bin
RUN pip3 install 'conan==2.*'
RUN apt -y install g++
