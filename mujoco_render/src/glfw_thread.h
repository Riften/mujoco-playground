//
// Created by yongxi on 2022/1/12.
//

#ifndef MUJOCO_PLAYGROUND_GLFW_THREAD_H
#define MUJOCO_PLAYGROUND_GLFW_THREAD_H

#include <thread>
#include <string>
#include <glfw3.h>

// GlfwThread is not accessible outside this library. So we do not add namespace here.

/**
 * A general limit of common window system is that you can only fetch window event from the same thread
 * create that window. In order to use a separate thread to show render result, the simplest way is to
 * collect all glfw related calls into the same thread.
 *
 * @note This class may not works on all operating system or window system. That is because some window
 * system may only send window event to the main thread.
 *
 * @note Only one GlfwThread instance at a time. Otherwise you should manage GL context properly.
 */
class GlfwThread: public std::thread {
public:
    explicit GlfwThread(const std::string& window_name = "MuJoCo", int window_width = 1200, int window_height = 900);
private:
    GLFWwindow* window_;
};

#endif //MUJOCO_PLAYGROUND_GLFW_THREAD_H
