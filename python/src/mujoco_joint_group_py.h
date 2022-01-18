//
// Created by yongxi on 2022/1/3.
//

#ifndef MUJOCO_PLAYGROUND_MUJOCO_JOINT_GROUP_PY_H
#define MUJOCO_PLAYGROUND_MUJOCO_JOINT_GROUP_PY_H

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <physics_mujoco/mujoco_joint_group.h>
#include <sstream>
namespace py = pybind11;

void bind_joint_group(py::module &m) {

    py::class_<physics_mujoco::JointGroup>(m, "JointGroup")
        .def(py::init<mjData *, physics_mujoco::KinematicTree &, const std::string& , const std::string&>())
        .def("__str__", [](const physics_mujoco::JointGroup& group){
            return physics_mujoco::KDLChainToString(group.chain());
        })
        .def("__repr__", [](const physics_mujoco::JointGroup& group){
            std::ostringstream sstr;
            sstr << "<JointGroup at " << &group << '>' << std::endl;
            sstr << physics_mujoco::KDLChainToString(group.chain());
            return sstr.str();
        })
        .def("getPos", [](const physics_mujoco::JointGroup& group) {
            double* data = group.getPos();
            py::capsule free_when_done(data, [](void *f) {
                auto data = reinterpret_cast<double*>(f);
                //std::cerr << "Free underlining numpy array memory @" << f << '\n';
                delete [] data;
            });
            return py::array_t<double>(
                    {group.size()},    //shape
                    {sizeof(double)}, //strides
                    data,                   //data pointer
                    free_when_done          //handler for free
            );
        });
}

#endif //MUJOCO_PLAYGROUND_MUJOCO_JOINT_GROUP_PY_H
