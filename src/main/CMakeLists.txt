find_package(imgui REQUIRED)
find_package(opengl_system REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(imgui-backend OBJECT
        imgui_impl_glfw.cpp
        imgui_impl_opengl3.cpp
)

target_compile_options(imgui-backend PUBLIC
        -Wno-unused-parameter
)

target_link_libraries(imgui-backend PUBLIC
        imgui::imgui
        opengl::opengl
)

add_executable(pong.html
        pong.cpp
)

target_link_options(pong.html PRIVATE
        "$<$<STREQUAL:${BUILD_PROFILE},emscripten>:-sUSE_GLFW=3>"
)

target_link_libraries(pong.html PRIVATE
        imgui-backend
        Eigen3::Eigen
        "$<$<STREQUAL:${BUILD_PROFILE},linux>:glfw>"
)
