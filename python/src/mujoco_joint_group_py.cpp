//
// Created by yongxi on 2022/1/3.
//

#include "mujoco_core_py.h"
#include "mujoco_kinematic_tree_py.h"
#include "mujoco_joint_group_py.h"


namespace py = pybind11;

PYBIND11_MODULE(_physics_mujoco, m) {
    bind_mujoco(m);
    bind_kinematic_tree(m);
    bind_joint_group(m);
}