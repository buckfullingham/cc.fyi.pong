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

add_library(pong-objects STATIC
        model.cpp
)

target_link_libraries(pong-objects PUBLIC
        Eigen3::Eigen
)

add_executable(pong.js
        pong.cpp
)

target_link_options(pong.js PRIVATE
        "$<$<STREQUAL:${BUILD_PROFILE},emscripten>:-sUSE_GLFW=3>"
)

target_link_libraries(pong.js PRIVATE
        imgui-backend
        "$<$<STREQUAL:${BUILD_PROFILE},linux>:glfw>"
        pong-objects
)

install(FILES
        ${CMAKE_CURRENT_BINARY_DIR}/pong.js
        "$<$<STREQUAL:${BUILD_PROFILE},emscripten>:${CMAKE_CURRENT_BINARY_DIR}/pong.wasm>"
        DESTINATION .
)

add_subdirectory(resources)
