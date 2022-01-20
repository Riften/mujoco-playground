//
// Created by yongxi on 2022/1/5.
//

#include <mujoco_render/mujoco_render.h>
#include <log4cxx/logger.h>

// Private headers
// #include "glfw_thread.h"

namespace mujoco_render {

    static log4cxx::LoggerPtr logger = log4cxx::Logger::getLogger("Render");

    Render::Render(mjModel *model, mjData *data, const std::string& window_name, int window_width, int window_height, bool paused ){
        // Initialize shared pointer
        mtx_ = std::make_shared<std::mutex>();
        paused_ = std::make_shared<bool>(paused);
        pause_condition_ = std::make_shared<std::condition_variable>();
        mj_forward(model, data);

        render_thread_ = new RenderThread(model, data, mtx_, pause_condition_, paused_, window_name, window_width, window_height);
        simulate_thread_ = new SimulateThread(model, data, mtx_, pause_condition_, paused_);
    }

    void Render::pause() {
        std::unique_lock<std::mutex> lock(*mtx_);
        *paused_ = !(*paused_);
        pause_condition_->notify_all();
    }

    Render::~Render() {
        // Free thread
        delete render_thread_;
        delete simulate_thread_;
    }

}