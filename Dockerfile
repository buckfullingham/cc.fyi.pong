FROM emscripten/emsdk

RUN apt install -vy \
    python3-venv \
    pkg-config \
    libopengl-dev \
    libglfw3-dev

RUN pip3 install \
    'conan==2.*'

