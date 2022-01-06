//
// Created by yongxi on 2022/1/5.
//

#ifndef MUJOCO_PLAYGROUND_MUJOCO_RENDER_H
#define MUJOCO_PLAYGROUND_MUJOCO_RENDER_H
#include <mujoco.h>
#include "glfw3.h"
#include <string>
#include <thread>
#include <atomic>

namespace mujoco_render {
    class Render; // forward decleration
    void render_thread_fn(Render* render);
    class Render {
    public:
        Render(mjModel* model,
               mjData* data,
               const std::string& window_name = "Render",
               int window_width = 1200,
               int window_height = 900,
               bool paused = true);
        ~Render();
    private:
        mjModel* mj_model_;
        mjData* mj_data_;
        mjvCamera mjv_camera_;                      // abstract camera
        mjvOption mjv_option_;                      // visualization options
        mjvScene mjv_scene_;                       // abstract scene
        mjrContext mjr_context_;                     // custom GPU context
        std::thread* render_thread_;
        GLFWwindow* window_;
        bool paused_;
        std::atomic<bool> interrupted_;  // An atomic variable used to interrupt render thread.
        /// @note argument of thread can not be reference.
        friend void render_thread_fn(Render* render);
    };
}

#endif //MUJOCO_PLAYGROUND_MUJOCO_RENDER_H
