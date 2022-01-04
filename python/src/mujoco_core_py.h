//
// Created by yongxi on 2022/1/3.
//

#ifndef MUJOCO_PLAYGROUND_MUJOCO_CORE_PY_H
#define MUJOCO_PLAYGROUND_MUJOCO_CORE_PY_H

#include <pybind11/pybind11.h>
#include <mujoco.h>
namespace py = pybind11;

/**
 * Bind mjModel and mjData
 * @param m
 */
void bind_mujoco(py::module &m) {
    py::class_<mjData> mjDataT(m, "mjData");
    py::class_<mjModel> mjModelT(m, "mjModel");
    mjModelT.def("make_data", [](mjModel& self){
            return mj_makeData(&self);
        });
    m.def("loadModelXML", [](const std::string& file_path) {
        char error[1000];
        return mj_loadXML(file_path.c_str(), nullptr, error, 1000);
    },py::arg("file_path"));

}

#endif //MUJOCO_PLAYGROUND_MUJOCO_CORE_PY_H
