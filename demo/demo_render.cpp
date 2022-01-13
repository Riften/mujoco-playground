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

#include <mujoco_render/mujoco_render.h>
#include <thread>


// Logger
log4cxx::LoggerPtr logger = log4cxx::Logger::getLogger("demo");


int main(int argc, char* argv[]) {
    physics_mujoco::initLogSystem();

    if( !glfwInit() )
        mju_error("Could not initialize GLFW");
    // char urdf_path[1000] = "/home/yongxi/Workspace/mujoco_playground/resources/panda_arm_hand/panda_arm_hand.urdf";
    char urdf_path[1000] = "/home/yongxi/Workspace/mujoco_playground/resources/panda_arm_hand/collision1.xml";
    // char urdf_path[1000] = "/home/yongxi/Workspace/mujoco_playground/resources/panda_arm_hand/panda_arm_hand.xml";
    char error[1000];

    mjModel * model = mj_loadXML(urdf_path, nullptr, error, 1000);

    if (!model) {
        std::cout << "can not load model: " << error << std::endl;
        return 1;
    }

    mjData * model_data = mj_makeData(model);

    mujoco_render::Render render(model, model_data);
    std::this_thread::sleep_for(10 * std::chrono::seconds());
    return 0;
}