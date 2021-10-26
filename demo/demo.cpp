//
// Created by yongxi on 2021/10/26.
//

#include <GL/glew.h>
#include <mujoco.h>
#include <glfw3.h>
#include <iostream>


int main(int argc, char* argv[]) {
    if( !glfwInit() )
        mju_error("Could not initialize GLFW");
    char urdf_path[1000] = "/home/yongxi/Workspace/mujoco_playground/resources/panda_arm_hand/panda_arm_hand.urdf";
    char error[1000];
    auto model = mj_loadXML(urdf_path, nullptr, error, 1000);

    if (!model) {
        std::cout << "can not load model: " << error << std::endl;
        return 1;
    }
    // make data
    auto model_data = mj_makeData(model);

    GLFWwindow* window = glfwCreateWindow(1200, 900, "Demo", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    mjvCamera cam;                      // abstract camera
    mjvOption opt;                      // visualization options
    mjvScene scn;                       // abstract scene
    mjrContext con;                     // custom GPU context

    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);

    // create scene and context
    mjv_makeScene(model, &scn, 2000);
    mjr_makeContext(model, &con, mjFONTSCALE_150);

    while( !glfwWindowShouldClose(window) )
    {
        // advance interactive simulation for 1/60 sec
        //  Assuming MuJoCo can simulate faster than real-time, which it usually can,
        //  this loop will finish on time for the next frame to be rendered at 60 fps.
        //  Otherwise add a cpu timer and exit this loop when it is time to render.
        mjtNum simstart = model_data->time;
        while( model_data->time - simstart < 1.0/60.0 )
            mj_step(model, model_data);

        // get framebuffer viewport
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

        // update scene and render
        mjv_updateScene(model, model_data, &opt, NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);

        // swap OpenGL buffers (blocking call due to v-sync)
        glfwSwapBuffers(window);

        // process pending GUI events, call GLFW callbacks
        glfwPollEvents();
    }

    return 0;
}