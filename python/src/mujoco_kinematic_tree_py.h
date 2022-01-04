//
// Created by yongxi on 2022/1/4.
//

#ifndef MUJOCO_PLAYGROUND_MUJOCO_KINEMATICS_TREE_PY_H
#define MUJOCO_PLAYGROUND_MUJOCO_KINEMATICS_TREE_PY_H

#include <pybind11/pybind11.h>
#include <physics_mujoco/mujoco_kinematic_tree.h>

namespace py = pybind11;

void bind_kinematic_tree(py::module & m) {
    py::class_<physics_mujoco::KinematicTree>(m, "KinematicTree")
        .def(py::init<const mjModel *, const std::string& , const std::string& , const std::string& >(),
            py::arg("mujoco_model"),
            py::arg("first_link"),
            py::arg("last_link"),
            py::arg("root_name")="kdl_root");
}

#endif //MUJOCO_PLAYGROUND_MUJOCO_KINEMATICS_TREE_PY_H
