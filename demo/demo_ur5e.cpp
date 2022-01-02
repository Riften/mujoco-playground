//
// Created by yongxi on 2021/10/26.
//

#include <GL/glew.h>
#include <mujoco.h>
#include <glfw3.h>
#include <iostream>
#include <physics_mujoco/mujoco_joint_controller.h>
#include <physics_mujoco/mujoco_joint_group.h>
#include <vector>
#include <thread>
#include <chrono>
// #include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <physics_mujoco/utils.h>
#include <physics_mujoco/mujoco_kinematic_tree.h>
#include <physics_mujoco/mujoco_state_validity_checker.h>
#include <log4cxx/logger.h>

// ompl
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>


// Logger
log4cxx::LoggerPtr logger = log4cxx::Logger::getLogger("demo");

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

// Joint Group
physics_mujoco::JointGroup* jointGroup = nullptr;

// OROCOS Kinematics
// KDL::Chain* kinematic_chain;
// KDL::ChainFkSolverPos* fk_solver;
KDL::Frame copied_tip;

// OMPL
ompl::geometric::SimpleSetupPtr simple_setup;
KDL::JntArray start_pos;
KDL::JntArray goal_pos;
ompl::geometric::PathGeometric* solution = nullptr;

// Show M
void showM() {
    mjtNum * index_vec = (mjtNum*) mju_malloc(sizeof(mjtNum) * model->nv);
    mjtNum * res_vec = (mjtNum*) mju_malloc(sizeof(mjtNum) * model->nv);

    std::fill_n(index_vec, model->nv, 1);

    mj_mulM(model, model_data, res_vec, index_vec);
    std::cout << "M for each joint: ";
    print_array(res_vec, model->nv);
    std::cout << std::endl;

    mju_free(index_vec);
    mju_free(res_vec);
}

void showFK() {
    auto eef_pos = jointGroup->eefPos();

    std::cout << "End effector:" << std::endl;
    std::cout << physics_mujoco::KDLFrameToString(eef_pos) << std::endl;

    std::cout << "From KDL:" << std::endl;
    auto kdl_fk = jointGroup->FK();
    std::cout << physics_mujoco::KDLFrameToString(kdl_fk) << std::endl;
    std::cout << "Difference:" << std::endl;
    std::cout << physics_mujoco::KDLVectorToString(eef_pos.p - kdl_fk.p) << std::endl;
}

void showContact() {
    std::cout << "Contacts: ";
    for(int i=0; i < model_data->ncon; ++i) {
        std::cout << '('
                  << mj_id2name(model, mjOBJ_BODY, model->geom_bodyid[model_data->contact[i].geom1])
                  << ','
                  << mj_id2name(model, mjOBJ_BODY, model->geom_bodyid[model_data->contact[i].geom2])
                  << ") ";
    }
    std::cout << std::endl;
}

void showSolution() {
    solution = &simple_setup->getSolutionPath();
    solution->interpolate(20);

    if(solution != nullptr) {
        KDL::JntArray tmp_pos(jointGroup->size());
        for(int i=0; i<solution->getStateCount(); ++i) {
            auto state = solution->getState(i)->as<ompl::base::RealVectorStateSpace::StateType>();
            for(int j=0; j<jointGroup->size(); ++j) {
                tmp_pos(j) = (*state)[j];
            }
            jointGroup->setPos(tmp_pos);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    }

}

void simpleMotionPlan() {
    LOG4CXX_DEBUG(logger, "Motion Plan from " << physics_mujoco::KDLJntArrayToString(start_pos) << " to " << physics_mujoco::KDLJntArrayToString(goal_pos));
    ompl::base::ScopedState<> start(simple_setup->getStateSpace());
    ompl::base::ScopedState<> goal(simple_setup->getStateSpace());
    for(int i=0; i<jointGroup->size(); ++i) {
        start[i] = start_pos(i);
        goal[i] = goal_pos(i);
    }
    //start.random();
    //goal.random();
    simple_setup->clear();
    simple_setup->clearStartStates();
    simple_setup->setStartAndGoalStates(start, goal);
    ompl::base::PlannerStatus solved = simple_setup->solve(1.0);
    if(solved) {
        simple_setup->simplifySolution();
        simple_setup->getSolutionPath().print(std::cout);
        std::thread th(showSolution);
        th.detach();
    } else {
        std::cout << "solution not found" << std::endl;
    }

}

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

        showM();
        showContact();
        if(jointGroup) {
            if(jointGroup->inCollision()) {
                std::cout << "Collision Detected" << std::endl;
            } else {
                std::cout << "Collision Free" << std::endl;
            }
            showFK();
        }
        // if(jointGroup.inCollision())
    }

    // Space: pause
    if( act==GLFW_PRESS && key==GLFW_KEY_SPACE) {
        paused = !paused;
    }

    // C: Copy current pose
    if( act == GLFW_PRESS && key == GLFW_KEY_C) {
        copied_tip = jointGroup->eefPos();
        LOG4CXX_DEBUG(logger, "Tip frame copied: " << physics_mujoco::KDLFrameToString(copied_tip));
    }

    // V: Compute IK and set pose
    if( act == GLFW_PRESS && key == GLFW_KEY_V) {
        KDL::JntArray ik_res = jointGroup->IK(copied_tip);
        KDL::JntArray after_ik;
        LOG4CXX_DEBUG(logger, "Set to ik res: " << physics_mujoco::KDLJntArrayToString(ik_res));
        jointGroup->setPos(ik_res);
        jointGroup->getPos(after_ik);
        LOG4CXX_DEBUG(logger, "After IK: " << physics_mujoco::KDLJntArrayToString(after_ik));
        KDL::Frame current_tip = jointGroup->eefPos();
        LOG4CXX_DEBUG(logger, "Difference: " << physics_mujoco::KDLVectorToString(copied_tip.p - current_tip.p));
    }

    // M: Motion example
    if( act == GLFW_PRESS && key == GLFW_KEY_M) {
        simpleMotionPlan();
    }

    // 1: Store current pos to start_pos
    if( act == GLFW_PRESS && key == GLFW_KEY_1) {
        ompl::base::ScopedState<> start(simple_setup->getStateSpace());
        start.random();
        for(int i = 0; i< jointGroup->size(); ++i) {
            start_pos(i) = start[i];
        }
        jointGroup->setPos(start_pos);
        LOG4CXX_DEBUG(logger, "Record start pos: " << physics_mujoco::KDLJntArrayToString(start_pos));
    }

    if( act == GLFW_PRESS && key == GLFW_KEY_2) {
        ompl::base::ScopedState<> goal(simple_setup->getStateSpace());
        goal.random();
        for(int i = 0; i< jointGroup->size(); ++i) {
            goal_pos(i) = goal[i];
        }
        jointGroup->setPos(goal_pos);
        LOG4CXX_DEBUG(logger, "Record goal pos: " << physics_mujoco::KDLJntArrayToString(goal_pos));
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

std::vector<physics_mujoco::JointController> controllers;

void damping_controller(const mjModel* m, mjData * d) {
    //if(m->nu == m->nv) {
    //    mju_scl(d->ctrl, d->qvel, -1, m->nv);
    //}
    // mju_scl(d->qfrc_applied, d->qvel, -2, m->nv);
    //mju_copy(d->qfrc_applied, d->qfrc_bias, m->nv);
    // if(!controllers.empty()) {
    //    controllers[0].setPos(0);
    //}
    for(auto& controller : controllers) {
        controller.setPos(1);
    }
}

int main(int argc, char* argv[]) {
    physics_mujoco::initLogSystem();

    if( !glfwInit() )
        mju_error("Could not initialize GL` FW");
    // char urdf_path[1000] = "/home/yongxi/Workspace/mujoco_playground/resources/panda_arm_hand/panda_arm_hand.urdf";
    // char urdf_path[1000] = "/home/yongxi/Workspace/mujoco_playground/resources/panda_arm_hand/collision1.xml";
    // char urdf_path[1000] = "/home/yongxi/Workspace/mujoco_playground/resources/panda_arm_hand/panda_arm_hand.xml";
    // char urdf_path[1000] = "/home/yongxi/Workspace/mujoco_playground/resources/ur5e/ur5e_robot.urdf";
    char urdf_path[1000] = "/home/yongxi/Workspace/mujoco_playground/resources/ur5e/collision1.xml";
    char error[1000];

    model = mj_loadXML(urdf_path, nullptr, error, 1000);

    if (!model) {
        std::cout << "can not load model: " << error << std::endl;
        return 1;
    }


    /**
    mj_saveLastXML("/home/yongxi/Workspace/mujoco_playground/resources/ur5e/ur5e_robot.xml",
                   model,
                   error,
                   1000);
    **/

    std::cout << "Joint Names:" << std::endl;
    for(int i=0; i<model->njnt; ++i) {
        std::cout << mj_id2name(model, mjOBJ_JOINT, i) << ", ";
    }
    std::cout << std::endl;

    std::cout << "Body Names:" << std::endl;
    for (int i=0; i< model->nbody; ++i) {
        std::cout << mj_id2name(model, mjOBJ_BODY, i) << ", ";
    }
    std::cout << std::endl;

    std::cout << "Number of position coordinates: " << model -> nq << std::endl;
    std::cout << "Number of DOFs: " << model->nv << std::endl;
    std::cout << "Number of actuators/controls: " << model -> nu << std::endl;


    //===============

    // make data
    model_data = mj_makeData(model);
    // std::cout << tree.kdl_tree().getRootSegment()->first << std::endl;



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

    // Set control callback
    // Simulate the start situation

    for(int i=0; i<model->nv-2; ++i) {
        model_data->qvel[i] = 0;
    }

    mj_step1(model, model_data);

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