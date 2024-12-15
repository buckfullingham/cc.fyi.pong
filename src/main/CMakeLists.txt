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

add_executable(pong.js
        pong.cpp
)

target_link_options(pong.js PRIVATE
        "$<$<STREQUAL:${BUILD_PROFILE},emscripten>:-sUSE_GLFW=3>"
)

target_link_libraries(pong.js PRIVATE
        imgui-backend
        Eigen3::Eigen
        "$<$<STREQUAL:${BUILD_PROFILE},linux>:glfw>"
)

install(FILES
        ${CMAKE_CURRENT_BINARY_DIR}/pong.js
        ${CMAKE_CURRENT_BINARY_DIR}/pong.wasm
        DESTINATION html
)

add_subdirectory(resources)
