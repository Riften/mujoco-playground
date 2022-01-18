//
// Created by yongxi on 2022/1/18.
//

#ifndef MUJOCO_PLAYGROUND_KDL_CORE_PY_H
#define MUJOCO_PLAYGROUND_KDL_CORE_PY_H

#include <pybind11/pybind11.h>
#include <kdl/jntarray.hpp>

namespace py=pybind11;

typedef Eigen::VectorXd::Scalar Scalar;
constexpr bool rowMajor = Eigen::VectorXd::Flags & Eigen::RowMajorBit;

void bind_kdl(py::module &m) {
    py::class_<KDL::JntArray>(m, "JntArray", py::buffer_protocol())
        .def_buffer([](KDL::JntArray& array)-> py::buffer_info{
            return py::buffer_info(
                array.data.data(),
                sizeof(Scalar),
                py::format_descriptor<Scalar>::format(),
                1,
                {array.rows(),},
                {1,}
            );
        });
}

#endif //MUJOCO_PLAYGROUND_KDL_CORE_PY_H
