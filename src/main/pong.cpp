#include "model.hpp"

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include <optional>
#include <random>

#define GL_SILENCE_DEPRECATION
#include <GLFW/glfw3.h> // Will drag system OpenGL headers

#ifdef __EMSCRIPTEN__
#include "emscripten_mainloop_stub.h"
#endif

static void glfw_error_callback(int, const char *) {
  // not emitting anything to the web browser
}

namespace {

inline Eigen::Vector2f vec(ImVec2 v) { return {v.x, v.y}; }

inline ImVec2 vec(Eigen::Vector2f v) { return {v(0), v(1)}; }

inline ImU32 col(Eigen::Matrix<std::uint8_t, 4, 1> v) {
  return IM_COL32(v(0), v(1), v(2), v(3));
}

struct settings_t {
  static constexpr int ai_skill_min = 5;
  static constexpr int ai_skill_default = 70;
  static constexpr int ai_skill_max = 95;
  int ai_skill = ai_skill_default;

  static constexpr float paddle_size_min = 20;
  static constexpr float paddle_size_default = 40;
  static constexpr float paddle_size_max = 60;
  float paddle_size = paddle_size_default;

  static constexpr float mouse_wheel_sensitivity_min = 1;
  static constexpr float mouse_wheel_sensitivity_default = 5;
  static constexpr float mouse_wheel_sensitivity_max = 20;
  float mouse_wheel_sensitivity = mouse_wheel_sensitivity_default;

  static constexpr int winning_score_min = 5;
  static constexpr int winning_score_default = 10;
  static constexpr int winning_score_max = 100;
  int winning_score = winning_score_default;

  friend bool operator==(const settings_t &, const settings_t &) = default;
};

} // namespace

int main(int, char **) {
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

  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO &io = ImGui::GetIO();
  io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
  io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;

  ImGui::StyleColorsDark();

  ImGui_ImplGlfw_InitForOpenGL(window, true);
#ifdef __EMSCRIPTEN__
  ImGui_ImplGlfw_InstallEmscriptenCallbacks(window, "#canvas");
#else
  ImGui_ImplGlfw_InstallCallbacks(window);
#endif
  ImGui_ImplOpenGL3_Init(glsl_version);

  const ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);

  settings_t settings;
  bool in_play = true;

  pong::arena_t arena{[prng = std::mt19937{std::random_device{}()},
                       theta_dist =
                           std::uniform_real_distribution<float>{
                               pong::constant::pi<float>() / 8.f,
                               pong::constant::pi<float>() * 3.f / 8.f},
                       quadrant_dist = std::uniform_int_distribution<int>{0, 3},
                       speed_dist = std::uniform_real_distribution<float>{
                           200, 300}]() mutable -> pong::vec_t {
    const pong::scalar_t quadrant =
        pong::scalar_t(quadrant_dist(prng)) * pong::constant::pi<float>() / 2.f;
    const pong::matrix_t rot =
        pong::transform::rot(theta_dist(prng) + quadrant);
    return rot * pong::unit::i * speed_dist(prng);
  }};

  std::optional<pong::ai_t> ai;
  ai.emplace(std::random_device{}(),
             (settings.paddle_size / 2.f + arena.puck().radius()) /
                 pong::z_scores[settings.ai_skill]);

#ifdef __EMSCRIPTEN__
  EMSCRIPTEN_MAINLOOP_BEGIN
#else
  while (!glfwWindowShouldClose(window))
#endif
  {
    glfwPollEvents();
    if (glfwGetWindowAttrib(window, GLFW_ICONIFIED) != 0) {
      ImGui_ImplGlfw_Sleep(10);
      continue;
    }

    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    if (ImGui::Begin("PONG", nullptr,
                     ImGuiWindowFlags_NoScrollbar |
                         ImGuiWindowFlags_NoResize)) {
      settings_t new_settings = settings;
      ImGui::SliderInt("AI skill", &new_settings.ai_skill,
                       settings_t::ai_skill_min, settings_t::ai_skill_max);
      ImGui::SliderFloat("Paddle size", &new_settings.paddle_size,
                         settings_t::paddle_size_min,
                         settings_t::paddle_size_max, "%.0f");
      ImGui::SliderFloat("Mouse wheel sensitivity",
                         &new_settings.mouse_wheel_sensitivity,
                         settings_t::mouse_wheel_sensitivity_min,
                         settings_t::mouse_wheel_sensitivity_max, "%.0f");
      ImGui::SliderInt("Winning score", &new_settings.winning_score,
                       settings_t::winning_score_min,
                       settings_t::winning_score_max);

      if (new_settings != std::exchange(settings, new_settings)) {
        arena.lhs_paddle().box().min()(1) =
            arena.centre()(1) - settings.paddle_size / 2.f;
        arena.lhs_paddle().box().max()(1) =
            arena.lhs_paddle().box().min()(1) + settings.paddle_size;
        arena.rhs_paddle().box().min()(1) =
            arena.centre()(1) - settings.paddle_size / 2.f;
        arena.rhs_paddle().box().max()(1) =
            arena.rhs_paddle().box().min()(1) + settings.paddle_size;
        ai.emplace(std::random_device{}(),
                   (settings.paddle_size / 2.f + arena.puck().radius()) /
                       pong::z_scores[settings.ai_skill]);
      }

      if (ImGui::Button("Reset scores")) {
        arena.lhs_score() = 0;
        arena.rhs_score() = 0;
      }

      if (ImGui::BeginChild("Arena", {640, 480})) {
        const auto draw_list = ImGui::GetWindowDrawList();
        const auto origin = vec(ImGui::GetCursorScreenPos());
        constexpr auto solid_white = IM_COL32(255, 255, 255, 255);

        if (const auto s = ai->paddle_speed(arena, arena.lhs_paddle())) {
          arena.lhs_paddle().velocity()(1) = *s;
        }

        arena.rhs_paddle().velocity()(1) = ImGui::GetIO().MouseWheel *
                                           settings.mouse_wheel_sensitivity /
                                           ImGui::GetIO().DeltaTime;

        if (in_play) {
          arena.advance_time(ImGui::GetIO().DeltaTime);
        }

        in_play = arena.lhs_score() < std::uint32_t(settings.winning_score) &&
                  arena.rhs_score() < std::uint32_t(settings.winning_score);

        // arena outline
        draw_list->AddRect(vec(origin + arena.box().min()),
                           vec(origin + arena.box().max()), solid_white, 5.f,
                           ImDrawFlags_RoundCornersAll);

        // centre line
        {
          pong::vec_t p1{arena.box().min()(0) +
                             (arena.box().max()(0) - arena.box().min()(0)) /
                                 2.f,
                         arena.box().min()(1)};
          pong::vec_t p2{p1(0), arena.box().max()(1)};
          draw_list->AddLine(vec(origin + p1), vec(origin + p2), solid_white);
        }

        // scores
        if (in_play) {
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
        } else {
          const std::string s = "WINNER!";
          const auto width = ImGui::CalcTextSize(&*s.begin(), &*s.end()).x;

          auto arena_width = (arena.box().max() - arena.box().min())(0);
          const auto x = arena.lhs_score() < arena.rhs_score()
                             ? origin(0) + arena.box().min()(0) +
                                   arena_width * .75f - width / 2.f
                             : origin(0) + arena.box().min()(0) +
                                   arena_width * .25f - width / 2.f;

          auto arena_height = (arena.box().max() - arena.box().min())(1);
          auto y = origin(1) + arena.box().min()(1) + arena_height * .125f;
          draw_list->AddText({x, y}, solid_white, &*s.begin(), &*s.end());
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
        ImGui::EndChild();
      }
      ImGui::End();
    }

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
