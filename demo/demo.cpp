//
// Created by yongxi on 2021/10/26.
//

#include <GL/glew.h>
#include <mujoco.h>
#include <glfw3.h>
#include <iostream>


// Util function
void print_array(mjtNum* arr, int len) {
    std::cout << "Size: " << len << " [ ";
    for(int i=0; i<len; ++i) {
        std::cout << arr[i]<<", ";
    }
    std::cout << ']';
}

// MuJoCo data structure
mjModel * model = nullptr;
mjData * model_data = nullptr;
mjvCamera cam;                      // abstract camera
mjvOption opt;                      // visualization options
mjvScene scn;                       // abstract scene
mjrContext con;                     // custom GPU context

// Global flags
bool paused = true;

// mouse interaction
bool button_left = false;
bool button_middle = false;
bool button_right =  false;
double lastx = 0;
double lasty = 0;

// keyboard callback
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods)
{
    // backspace: reset simulation
    if( act==GLFW_PRESS && key==GLFW_KEY_BACKSPACE )
    {
        mj_resetData(model, model_data);
        mj_forward(model, model_data);
    }

    // P: print info
    if( act==GLFW_PRESS && key==GLFW_KEY_P) {
        std::cout << "================" << std::endl;
        std::cout << "Number of active constraints: " << model_data->nefc << std::endl;
        std::cout << "Joint Position:" << std::endl;
        print_array(model_data->qpos, model->nq);
        std::cout << std::endl;

        std::cout << "Joint Velocity:" << std::endl;
        print_array(model_data->qvel, model->nv);
        std::cout << std::endl;

        model_data->qM;
    }

    // Space: pause
    if( act==GLFW_PRESS && key==GLFW_KEY_SPACE) {
        paused = !paused;
    }
}

// scroll callback
void scroll(GLFWwindow* window, double xoffset, double yoffset)
{
    // emulate vertical mouse motion = 5% of window height
    mjv_moveCamera(model, mjMOUSE_ZOOM, 0, 0.05*yoffset, &scn, &cam);
}

// mouse button callback
void mouse_button(GLFWwindow* window, int button, int act, int mods)
{
    // update button state
    button_left =   (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
    button_right =  (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);

    // update mouse position
    glfwGetCursorPos(window, &lastx, &lasty);
}

// mouse move callback
void mouse_move(GLFWwindow* window, double xpos, double ypos)
{
    // no buttons down: nothing to do
    if( !button_left && !button_middle && !button_right )
        return;

    // compute mouse displacement, save
    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    // get current window size
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    // get shift key state
    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);

    // determine action based on mouse button
    mjtMouse action;
    if( button_right )
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    else if( button_left )
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else
        action = mjMOUSE_ZOOM;

    // move camera
    mjv_moveCamera(model, action, dx/height, dy/height, &scn, &cam);
}

int main(int argc, char* argv[]) {
    if( !glfwInit() )
        mju_error("Could not initialize GL` FW");
    // char urdf_path[1000] = "/home/yongxi/Workspace/mujoco_playground/resources/panda_arm_hand/panda_arm_hand.urdf";
    char urdf_path[1000] = "/home/yongxi/Workspace/mujoco_playground/resources/panda_arm_hand/collision1.xml";
    // char urdf_path[1000] = "/home/yongxi/Workspace/mujoco_playground/resources/panda_arm_hand/panda_arm_hand.xml";
    char error[1000];
    model = mj_loadXML(urdf_path, nullptr, error, 1000);

    if (!model) {
        std::cout << "can not load model: " << error << std::endl;
        return 1;
    }

    /**
    mj_saveLastXML("/home/yongxi/Workspace/mujoco_playground/resources/panda_arm_hand.xml",
                   model,
                   error,
                   1000);
    */

    std::cout << "Joint Names:" << std::endl;
    for(int i=0; i<model->njnt; ++i) {
        std::cout << mj_id2name(model, mjOBJ_JOINT, i) << std::endl;
    }

    std::cout << "Number of position coordinates: " << model -> nq << std::endl;
    std::cout << "Number of DOFs: " << model->nv << std::endl;


    // make data
    model_data = mj_makeData(model);

    GLFWwindow* window = glfwCreateWindow(1200, 900, "Demo", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);

    // create scene and context
    mjv_makeScene(model, &scn, 2000);
    mjr_makeContext(model, &con, mjFONTSCALE_150);

    // Set callback
    glfwSetKeyCallback(window, keyboard);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetScrollCallback(window, scroll);

    // Simulate the start situation
    mj_step(model, model_data);

    while( !glfwWindowShouldClose(window) )
    {
        // advance interactive simulation for 1/60 sec
        //  Assuming MuJoCo can simulate faster than real-time, which it usually can,
        //  this loop will finish on time for the next frame to be rendered at 60 fps.
        //  Otherwise add a cpu timer and exit this loop when it is time to render.
        mjtNum simstart = model_data->time;

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

        if(!paused) {
            while (model_data->time - simstart < 1.0 / 60.0)
                mj_step(model, model_data);
        }
    }

    return 0;
}