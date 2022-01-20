//
// Created by yongxi on 2022/1/3.
//

#include "kdl_core_py.h"
#include "mujoco_core_py.h"
#include "mujoco_kinematic_tree_py.h"
#include "mujoco_joint_group_py.h"
#include "mujoco_render_py.h"
#include <physics_mujoco/utils.h>
#include <mutex>

namespace py = pybind11;

PYBIND11_MODULE(_physics_mujoco, m) {
    m.def("initLogSystem", physics_mujoco::initLogSystem);

    // Bind mutex with shared_ptr as holder
    py::class_<std::mutex, std::shared_ptr<std::mutex>>(m, "Mutex");
    bind_kdl(m);
    bind_mujoco(m);
    bind_mujoco_render(m);
    bind_kinematic_tree(m);
    bind_joint_group(m);
}