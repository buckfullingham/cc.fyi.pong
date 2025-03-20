# emscripten deploy their arm64 images under specific tags
ARG EMSDK_IMAGE_TAG=latest
FROM emscripten/emsdk:${EMSDK_IMAGE_TAG}

RUN apt -y update \
 && apt -y upgrade \
 && apt -y install \
    python3-venv

RUN python3 -m venv /venv
ENV PATH=$PATH:/venv/bin
COPY ../requirements.txt /tmp/
RUN pip3 install -U -r /tmp/requirements.txt && rm /tmp/requirements.txt

ENV BUILD_PROFILE=emscripten
CMD /bin/bash
