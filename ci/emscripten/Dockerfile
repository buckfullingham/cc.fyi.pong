FROM emscripten/emsdk

RUN apt -y update \
 && apt -y upgrade \
 && apt -y install \
    python3-venv

RUN python3 -m venv /venv
ENV PATH=$PATH:/venv/bin
COPY ../requirements.txt /tmp/
RUN pip3 install -U -r /tmp/requirements.txt && rm /tmp/requirements.txt

ENV BUILD_PROFILE=emscripten
# # cf https://askubuntu.com/questions/355565/how-do-i-install-the-latest-version-of-cmake-from-the-command-line
# RUN true \
#  && apt -y install gpg software-properties-common lsb-release \
#  && apt -y purge --auto-remove cmake \
#  && wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | tee /etc/apt/trusted.gpg.d/kitware.gpg >/dev/null \
#  && apt-add-repository -y "deb https://apt.kitware.com/ubuntu/ $(lsb_release -cs) main" \
#  && apt -y update \
#  && apt -y install kitware-archive-keyring \
#  && rm -f /etc/apt/trusted.gpg.d/kitware.gpg \
#  && apt -y install cmake

RUN useradd pong
USER pong
