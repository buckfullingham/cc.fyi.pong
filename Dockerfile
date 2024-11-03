FROM emscripten/emsdk

RUN apt install -vy \
    python3-venv

RUN pip3 install \
    'conan==2.*'

