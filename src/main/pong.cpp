// Dear ImGui: standalone example application for GLFW + OpenGL 3, using
// programmable pipeline (GLFW is a cross-platform general purpose library for
// handling windows, inputs, OpenGL/Vulkan/Metal graphics context creation,
// etc.)

// Learn about Dear ImGui:
// - FAQ                  https://dearimgui.com/faq
// - Getting Started      https://dearimgui.com/getting-started
// - Documentation        https://dearimgui.com/docs (same as your local docs/
// folder).
// - Introduction, links and more at the top of imgui.cpp

#include "model.hpp"

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include <optional>
#include <random>

#include <stdio.h>
#define GL_SILENCE_DEPRECATION
#if defined(IMGUI_IMPL_OPENGL_ES2)
#include <GLES2/gl2.h>
#endif
#include <GLFW/glfw3.h> // Will drag system OpenGL headers

// [Win32] Our example includes a copy of glfw3.lib pre-compiled with VS2010 to
// maximize ease of testing and compatibility with old VS compilers. To link
// with VS2010-era libraries, VS2015+ requires linking with
// legacy_stdio_definitions.lib, which we do using this pragma. Your own project
// should not be affected, as you are likely to link with a newer binary of GLFW
// that is adequate for your version of Visual Studio.
#if defined(_MSC_VER) && (_MSC_VER >= 1900) &&                                 \
    !defined(IMGUI_DISABLE_WIN32_FUNCTIONS)
#pragma comment(lib, "legacy_stdio_definitions")
#endif

// This example can also compile and run with Emscripten! See
// 'Makefile.emscripten' for details.
#ifdef __EMSCRIPTEN__
#include "emscripten_mainloop_stub.h"
#endif

static void glfw_error_callback(int error, const char *description) {
  fprintf(stderr, "GLFW Error %d: %s\n", error, description);
}

namespace {

inline Eigen::Vector2f vec(ImVec2 v) { return {v.x, v.y}; }

inline ImVec2 vec(Eigen::Vector2f v) { return {v(0), v(1)}; }

inline ImU32 col(Eigen::Matrix<std::uint8_t, 4, 1> v) {
  return IM_COL32(v(0), v(1), v(2), v(3));
}

// inline Eigen::Vector4f vec(ImVec4 v) {
//   return {v.x, v.y, v.z, v.w};
// }

} // namespace

// Main code
int main(int, char **) {
  std::cout << "starting..." << std::endl;
  glfwSetErrorCallback(glfw_error_callback);
  if (!glfwInit())
    return 1;

  // GL ES 2.0 + GLSL 100
  const char *glsl_version = "#version 100";
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
  glfwWindowHint(GLFW_CLIENT_API, GLFW_OPENGL_ES_API);

  // Create window with graphics context
  GLFWwindow *window = glfwCreateWindow(1280, 720, "PONG", nullptr, nullptr);
  if (window == nullptr)
    return 1;
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1); // Enable vsync

  // Setup Dear ImGui context
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO &io = ImGui::GetIO();
  (void)io;
  io.ConfigFlags |=
      ImGuiConfigFlags_NavEnableKeyboard; // Enable Keyboard Controls
  io.ConfigFlags |=
      ImGuiConfigFlags_NavEnableGamepad; // Enable Gamepad Controls

  ImGui::StyleColorsDark();

  // Setup Platform/Renderer backends
  ImGui_ImplGlfw_InitForOpenGL(window, true);
#ifdef __EMSCRIPTEN__
  ImGui_ImplGlfw_InstallEmscriptenCallbacks(window, "#canvas");
#else
  ImGui_ImplGlfw_InstallCallbacks(window);
#endif
  ImGui_ImplOpenGL3_Init(glsl_version);

  // Load Fonts
  // - If no fonts are loaded, dear imgui will use the default font. You can
  // also load multiple fonts and use ImGui::PushFont()/PopFont() to select
  // them.
  // - AddFontFromFileTTF() will return the ImFont* so you can store it if you
  // need to select the font among multiple.
  // - If the file cannot be loaded, the function will return a nullptr. Please
  // handle those errors in your application (e.g. use an assertion, or display
  // an error and quit).
  // - The fonts will be rasterized at a given size (w/ oversampling) and stored
  // into a texture when calling ImFontAtlas::Build()/GetTexDataAsXXXX(), which
  // ImGui_ImplXXXX_NewFrame below will call.
  // - Use '#define IMGUI_ENABLE_FREETYPE' in your imconfig file to use Freetype
  // for higher quality font rendering.
  // - Read 'docs/FONTS.md' for more instructions and details.
  // - Remember that in C/C++ if you want to include a backslash \ in a string
  // literal you need to write a double backslash \\ !
  // - Our Emscripten build process allows embedding fonts to be accessible at
  // runtime from the "fonts/" folder. See Makefile.emscripten for details.
  // io.Fonts->AddFontDefault();
  // io.Fonts->AddFontFromFileTTF("c:\\Windows\\Fonts\\segoeui.ttf", 18.0f);
  // io.Fonts->AddFontFromFileTTF("../../misc/fonts/DroidSans.ttf", 16.0f);
  // io.Fonts->AddFontFromFileTTF("../../misc/fonts/Roboto-Medium.ttf", 16.0f);
  // io.Fonts->AddFontFromFileTTF("../../misc/fonts/Cousine-Regular.ttf", 15.0f);
  // ImFont* font =
  // io.Fonts->AddFontFromFileTTF("c:\\Windows\\Fonts\\ArialUni.ttf", 18.0f,
  // nullptr, io.Fonts->GetGlyphRangesJapanese()); IM_ASSERT(font != nullptr);

  // Our state
  ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);

  pong::arena_t arena{[prng = std::mt19937{std::random_device{}()},
                       theta_dist =
                           std::uniform_real_distribution<float>{
                               pong::constant::pi<float>() / 8.f,
                               pong::constant::pi<float>() / 4.f},
                       sign_dist = std::uniform_int_distribution<int>{0, 1},
                       speed_dist = std::uniform_real_distribution<float>{
                           200, 300}]() mutable -> pong::vec_t {
    const pong::scalar_t theta = theta_dist(prng);
    const pong::matrix_t signs{
        {pong::scalar_t(sign_dist(prng) * 2 - 1), 0.f},
        {0.f, pong::scalar_t(sign_dist(prng) * 2 - 1)},
    };
    return pong::transform::rot(theta) * signs * pong::unit::i *
           speed_dist(prng);
  }};

  // Main loop
#ifdef __EMSCRIPTEN__
  //   For an Emscripten build we are disabling file-system access, so let's not
  //   attempt to do a fopen() of the imgui.ini file. You may manually call
  //   LoadIniSettingsFromMemory() to load settings from your own storage.
  //  io.IniFilename = nullptr;
  EMSCRIPTEN_MAINLOOP_BEGIN
#else
  while (!glfwWindowShouldClose(window))
#endif
  {
    // Poll and handle events (inputs, window resize, etc.)
    // You can read the io.WantCaptureMouse, io.WantCaptureKeyboard flags to
    // tell if dear imgui wants to use your inputs.
    // - When io.WantCaptureMouse is true, do not dispatch mouse input data to
    // your main application, or clear/overwrite your copy of the mouse data.
    // - When io.WantCaptureKeyboard is true, do not dispatch keyboard input
    // data to your main application, or clear/overwrite your copy of the
    // keyboard data. Generally you may always pass all inputs to dear imgui,
    // and hide them from your application based on those two flags.
    glfwPollEvents();
    if (glfwGetWindowAttrib(window, GLFW_ICONIFIED) != 0) {
      ImGui_ImplGlfw_Sleep(10);
      continue;
    }

    // Start the Dear ImGui frame
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    // 1. Show the big demo window (Most of the sample code is in
    // ImGui::ShowDemoWindow()! You can browse its code to learn more about Dear
    // ImGui!).

    // 2. Show a simple window that we create ourselves. We use a Begin/End pair
    // to create a named window.

    {
      ImGui::SetNextWindowContentSize({640, 480});
      ImGui::Begin("Arena", nullptr,
                   ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoResize);
      auto draw_list = ImGui::GetWindowDrawList();

      auto origin = vec(ImGui::GetCursorScreenPos());

      constexpr auto solid_white = IM_COL32(255, 255, 255, 255);

      arena.rhs_paddle().velocity()(1) =
          ImGui::GetIO().MouseWheel * 5.f / ImGui::GetIO().DeltaTime;

      arena.advance_time(ImGui::GetIO().DeltaTime);

      // arena outline
      draw_list->AddRect(vec(origin + arena.box().min()),
                         vec(origin + arena.box().max()), solid_white, 5.f,
                         ImDrawFlags_RoundCornersAll);

      // centre line
      {
        pong::vec_t p1{arena.box().min()(0) +
                           (arena.box().max()(0) - arena.box().min()(0)) / 2.f,
                       arena.box().min()(1)};
        pong::vec_t p2{p1(0), arena.box().max()(1)};
        draw_list->AddLine(vec(origin + p1), vec(origin + p2), solid_white);
      }

      // scores
      {
        auto lhs_score = std::to_string(arena.lhs_score());
        auto rhs_score = std::to_string(arena.rhs_score());
        auto lhs_width =
            ImGui::CalcTextSize(&*lhs_score.begin(), &*lhs_score.end()).x;
        auto rhs_width =
            ImGui::CalcTextSize(&*rhs_score.begin(), &*rhs_score.end()).x;
        auto arena_width = (arena.box().max() - arena.box().min())(0);
        auto arena_height = (arena.box().max() - arena.box().min())(1);
        auto lhs_x = origin(0) + arena.box().min()(0) + arena_width * .25f -
                     lhs_width / 2.f;
        auto rhs_x = origin(0) + arena.box().min()(0) + arena_width * .75f -
                     rhs_width / 2.f;
        auto y = origin(1) + arena.box().min()(1) + arena_height * .125f;
        draw_list->AddText({lhs_x, y}, solid_white, &*lhs_score.begin(),
                           &*lhs_score.end());
        draw_list->AddText({rhs_x, y}, solid_white, &*rhs_score.begin(),
                           &*rhs_score.end());
      }

      // puck
      draw_list->AddCircleFilled(vec(origin + arena.puck().centre()),
                                 arena.puck().radius(),
                                 col(arena.puck().colour()));

      // lhs paddle
      draw_list->AddRectFilled(vec(origin + arena.lhs_paddle().box().min()),
                               vec(origin + arena.lhs_paddle().box().max()),
                               col(arena.lhs_paddle().colour()));

      // rhs paddle
      draw_list->AddRectFilled(vec(origin + arena.rhs_paddle().box().min()),
                               vec(origin + arena.rhs_paddle().box().max()),
                               col(arena.rhs_paddle().colour()));
      ImGui::End();
    }

    // Rendering
    ImGui::Render();
    int display_w, display_h;
    glfwGetFramebufferSize(window, &display_w, &display_h);
    glViewport(0, 0, display_w, display_h);
    glClearColor(clear_color.x * clear_color.w, clear_color.y * clear_color.w,
                 clear_color.z * clear_color.w, clear_color.w);
    glClear(GL_COLOR_BUFFER_BIT);
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    glfwSwapBuffers(window);
  }
#ifdef __EMSCRIPTEN__
  EMSCRIPTEN_MAINLOOP_END;
#endif

  // Cleanup
  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();

  glfwDestroyWindow(window);
  glfwTerminate();

  return 0;
}
