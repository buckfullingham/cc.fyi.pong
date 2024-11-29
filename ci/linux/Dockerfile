FROM ubuntu

RUN apt -y update \
 && apt -y upgrade \
 && apt -y install \
    clang \
    cmake \
    python3-full \
    libgl-dev \
    pkg-config \
    gdb \
    g++

RUN python3 -m venv /venv
ENV PATH=$PATH:/venv/bin
COPY ../requirements.txt /tmp/
RUN pip3 install -U -r /tmp/requirements.txt && rm /tmp/requirements.txt
