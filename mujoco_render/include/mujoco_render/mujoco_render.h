//
// Created by yongxi on 2022/1/5.
//

#ifndef MUJOCO_PLAYGROUND_MUJOCO_RENDER_H
#define MUJOCO_PLAYGROUND_MUJOCO_RENDER_H
#include <mujoco.h>
#include <string>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <mujoco_render/render_thread.h>
#include <mujoco_render/simulate_thread.h>

namespace mujoco_render {

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
        RenderThread* render_thread_ = nullptr;
        SimulateThread* simulate_thread_ = nullptr;

        // It is convenient to use shared pointers here as we do not need to free them manually
        // which could be confusing as it is used through different thread.
        std::shared_ptr<std::mutex> mtx_;
        std::shared_ptr<std::condition_variable> pause_condition_;
        std::shared_ptr<bool> paused_;
    };
}

#endif //MUJOCO_PLAYGROUND_MUJOCO_RENDER_H
