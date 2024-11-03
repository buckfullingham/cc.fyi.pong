find_package(opengl_system REQUIRED)

add_executable(pong.html
        pong.cpp
        esUtil.cpp
)

target_link_libraries(
        pong.html PRIVATE
        opengl::opengl
)

target_link_options(
        pong.html PRIVATE
        -sMIN_WEBGL_VERSION=2
        -sMAX_WEBGL_VERSION=2
)
