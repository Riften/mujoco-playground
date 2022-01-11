//GLFWwindow* window_;
// Created by yongxi on 2022/1/12.
//

#include "glfw_thread.h"
#include <log4cxx/logger.h>

static log4cxx::LoggerPtr logger = log4cxx::Logger::getLogger("GlfwThread");

GlfwThread::GlfwThread(const std::string& window_name, int window_width, int window_height) {
    if( !glfwInit() ) {
        LOG4CXX_FATAL(logger, "Could not initialize GL` FW");
    }
    window_ = glfwCreateWindow(window_width, window_height, window_name.c_str(), NULL, NULL);
    glfwMakeContextCurrent(window_);
    glfwSwapInterval(1);
}