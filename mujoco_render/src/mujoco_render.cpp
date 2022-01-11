//
// Created by yongxi on 2022/1/5.
//

#include <mujoco_render/mujoco_render.h>
#include <log4cxx/logger.h>

// Private headers
#include "glfw_thread.h"
#include "render_thread.h"

namespace mujoco_render {

    static log4cxx::LoggerPtr logger = log4cxx::Logger::getLogger("Render");

    Render::Render(mjModel *model, mjData *data, const std::string& window_name, int window_width, int window_height, bool paused )
    : mj_model_(model)
    , mj_data_(data)
    , paused_(paused){
        if( !glfwInit() ) {
            LOG4CXX_FATAL(logger, "Could not initialize GL` FW");
        }
        window_ = glfwCreateWindow(window_width, window_height, window_name.c_str(), NULL, NULL);
        glfwMakeContextCurrent(window_);
        glfwSwapInterval(1);

        mjv_defaultCamera(&mjv_camera_);
        mjv_defaultOption(&mjv_option_);
        mjv_defaultScene(&mjv_scene_);
        mjr_defaultContext(&mjr_context_);

        // create scene and context
        // @todo make maxgeom and fontscale configurable
        mjv_makeScene(model, &mjv_scene_, 2000);
        mjr_makeContext(model, &mjr_context_, mjFONTSCALE_150);

        interrupted_ = false;
        // render_thread_ = new std::thread(render_thread_fn, this);

        mj_step(mj_model_, mj_data_);
        while( !glfwWindowShouldClose(window_) && !interrupted_)
        {
            // advance interactive simulation for 1/60 sec
            //  Assuming MuJoCo can simulate faster than real-time, which it usually can,
            //  this loop will finish on time for the next frame to be rendered at 60 fps.
            //  Otherwise add a cpu timer and exit this loop when it is time to render.
            mjtNum simstart = mj_data_->time;

            // get framebuffer viewport
            mjrRect viewport = {0, 0, 0, 0};
            glfwGetFramebufferSize(window_, &viewport.width, &viewport.height);

            // update scene and render
            mjv_updateScene(mj_model_, mj_data_, &mjv_option_, NULL,
                            &mjv_camera_, mjCAT_ALL, &mjv_scene_);
            mjr_render(viewport, &mjv_scene_, &mjr_context_);

            // swap OpenGL buffers (blocking call due to v-sync)
            glfwSwapBuffers(window_);

            // process pending GUI events, call GLFW callbacks
            glfwPollEvents();

            if(!paused_) {
                while (mj_data_->time - simstart < 1.0 / 60.0)
                    mj_step(mj_model_, mj_data_);
            }
        }

    }

    Render::~Render() {
        // Free thread
        LOG4CXX_DEBUG(logger, "Wait for render thread to stop.")
        interrupted_ = true;
        if(render_thread_->joinable()) {
            render_thread_->join();
            delete render_thread_;
        } else {
            LOG4CXX_DEBUG(logger, "render thread not joinable");
        }
        mjv_freeScene(&mjv_scene_);
        mjr_freeContext(&mjr_context_);
    }
    void render_thread_fn(Render* render) {
        LOG4CXX_DEBUG(logger, "Mujoco Render Start");
        mj_step(render->mj_model_, render->mj_data_);
        while( !glfwWindowShouldClose(render->window_) && !render->interrupted_)
        {
            // advance interactive simulation for 1/60 sec
            //  Assuming MuJoCo can simulate faster than real-time, which it usually can,
            //  this loop will finish on time for the next frame to be rendered at 60 fps.
            //  Otherwise add a cpu timer and exit this loop when it is time to render.
            mjtNum simstart = render->mj_data_->time;

            // get framebuffer viewport
            mjrRect viewport = {0, 0, 0, 0};
            glfwGetFramebufferSize(render->window_, &viewport.width, &viewport.height);

            // update scene and render
            mjv_updateScene(render->mj_model_, render->mj_data_, &render->mjv_option_, NULL,
                            &render->mjv_camera_, mjCAT_ALL, &render->mjv_scene_);
            mjr_render(viewport, &(render->mjv_scene_), &(render->mjr_context_));

            // swap OpenGL buffers (blocking call due to v-sync)
            glfwSwapBuffers(render->window_);

            // process pending GUI events, call GLFW callbacks
            glfwPollEvents();

            if(!render->paused_) {
                while (render->mj_data_->time - simstart < 1.0 / 60.0)
                    mj_step(render->mj_model_, render->mj_data_);
            }
        }
        LOG4CXX_DEBUG(logger, "Mujoco Render End");
    }
}