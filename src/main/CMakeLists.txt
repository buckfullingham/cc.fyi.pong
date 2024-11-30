find_package(imgui REQUIRED)
find_package(opengl_system REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(glfw3 REQUIRED)

add_library(
        imgui-backend OBJECT
        imgui_impl_glfw.cpp
        imgui_impl_opengl3.cpp
)

target_link_libraries(
        imgui-backend PUBLIC
        imgui::imgui
        opengl::opengl
        #        glfw
)

target_compile_options(
        imgui-backend PUBLIC
        -Wno-unused-parameter
)

if (EMSCRIPTEN)
    target_link_options(
            imgui-backend PUBLIC
            -sUSE_GLFW=3
    )
else()
    target_link_libraries(
            imgui-backend PUBLIC
            glfw
    )
endif()

add_executable(pong.html
        pong.cpp
)

target_link_libraries(
        pong.html PRIVATE
        imgui-backend
        Eigen3::Eigen
)
