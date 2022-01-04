//
// Created by yongxi on 2022/1/3.
//

#ifndef MUJOCO_PLAYGROUND_MUJOCO_JOINT_GROUP_PY_H
#define MUJOCO_PLAYGROUND_MUJOCO_JOINT_GROUP_PY_H

#include <pybind11/pybind11.h>
#include <physics_mujoco/mujoco_joint_group.h>

namespace py = pybind11;

void bind_joint_group(py::module &m) {
    py::class_<physics_mujoco::JointGroup>(m, "JointGroup")
        .def(py::init<mjData *, physics_mujoco::KinematicTree &, const std::string& , const std::string&>());
}

#endif //MUJOCO_PLAYGROUND_MUJOCO_JOINT_GROUP_PY_H
