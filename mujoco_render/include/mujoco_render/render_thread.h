//
// Created by yongxi on 2022/1/12.
//

#ifndef MUJOCO_PLAYGROUND_RENDER_THREAD_H
#define MUJOCO_PLAYGROUND_RENDER_THREAD_H

#include <thread>
#include <string>
#include <glfw3.h>
#include <mutex>
#include <chrono>
#include <log4cxx/logger.h>

namespace mujoco_render {

/**
 * A general limit of common window system is that you can only fetch window event from the same thread
 * create that window. In order to use a separate thread to show render result, the simplest way is to
 * collect all glfw related calls into the same thread.
 *
 * @note This class may not works on all operating system or window system. That is because some window
 * system may only send window event to the main thread.
 *
 * @note Only one RenderThread instance at a time. Otherwise you should manage GL context properly.
 *
 * @note It is really a bad idea to derive std::thread directly.
 *
 * @todo Make more parameters configurable.
 */
    class RenderThread {
    public:
        typedef std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> TimePoint;

        explicit RenderThread(mjModel *model, mjData *data, std::mutex *mtx, const std::string &window_name = "MuJoCo",
                              int window_width = 1200, int window_height = 900)
                : mtx_(mtx)
                , window_height_(window_height)
                , window_width_(window_width)
                , window_name_(window_name){
            logger = log4cxx::Logger::getLogger("RenderThread");

            init_mujoco(model, data);

            // Start rendering thread
            thread_ = new std::thread([this]() {
                this->render_loop();
            });

            // Destroy window
            // glfwDestroyWindow(window_);
        }

        /**
         * @note glfwSetWindowShouldClose() can be called from any thread.
         */
        void exit() {
            // Close window
            glfwSetWindowShouldClose(window_, 1);
        }

        ~RenderThread() {
            // Terminate thread
            LOG4CXX_DEBUG(logger, "Terminate rendering thread");
            glfwSetWindowShouldClose(window_, 1);
            if (thread_->joinable()) {
                thread_->join();
            }
            delete thread_;

            // Free mujoco render variable
            LOG4CXX_DEBUG(logger, "Free mujoco render variable");
            //mjv_freeScene(&scene_);
            //mjr_freeContext(&context_);
        }

    private:
        void init_mujoco(mjModel *model, mjData *data) {
            model_ = model;
            model_data_ = data;

            mtx_->lock();
            mjv_defaultCamera(&camera_);
            mjv_defaultOption(&option_);
            mjv_defaultScene(&scene_);
            mjr_defaultContext(&context_);
            mtx_->unlock();
            // create scene and context

        }

        void render_loop() {
            if (!glfwInit()) {
                LOG4CXX_FATAL(logger, "Could not initialize GLFW");
            }
            window_ = glfwCreateWindow(window_width_, window_height_, window_name_.c_str(), NULL, NULL);

            glfwMakeContextCurrent(window_);
            glfwSwapInterval(1); // SwapInterval is the interval monitor frames when rendering.

            LOG4CXX_DEBUG(logger, "Start render loop");
            mtx_->lock();
            mjv_makeScene(model_, &scene_, 2000);
            mjr_makeContext(model_, &context_, mjFONTSCALE_150);
            mtx_->unlock();
            TimePoint next = std::chrono::steady_clock::now();
            auto interval = 1000 * std::chrono::milliseconds() / 60;

            while (!glfwWindowShouldClose(window_)) {
                // get framebuffer viewport
                mjrRect viewport = {0, 0, 0, 0};
                glfwGetFramebufferSize(window_, &viewport.width, &viewport.height);

                // update scene and render
                mtx_->lock();
                mjv_updateScene(model_, model_data_, &option_, NULL, &camera_, mjCAT_ALL, &scene_);
                mtx_->unlock();

                mjr_render(viewport, &scene_, &context_);


                // swap OpenGL buffers (blocking call due to v-sync)
                glfwSwapBuffers(window_);

                // process pending GUI events, call GLFW callbacks
                glfwPollEvents();
                next += interval;
                std::this_thread::sleep_until(next);
            }
            glfwDestroyWindow(window_);
            mtx_->lock();
            mjv_freeScene(&scene_);
            mjr_freeContext(&context_);
            mtx_->unlock();
            LOG4CXX_DEBUG(logger, "Render loop ends");
        }

        GLFWwindow *window_;
        log4cxx::LoggerPtr logger;
        std::mutex *mtx_;
        int window_height_;
        int window_width_;
        std::string window_name_;

        // thread instance
        std::thread *thread_;

        // mujoco variables
        mjModel *model_ = nullptr;
        mjData *model_data_ = nullptr;
        mjvCamera camera_;                      // abstract camera
        mjvOption option_;                      // visualization options
        mjvScene scene_;                       // abstract scene
        mjrContext context_;                     // custom GPU context

        // Rate control

    };
}

#endif //MUJOCO_PLAYGROUND_RENDER_THREAD_H
