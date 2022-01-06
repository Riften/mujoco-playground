//
// Created by yongxi on 2022/1/6.
//

#ifndef MUJOCO_PLAYGROUND_MUJOCO_RENDER_PY_H
#define MUJOCO_PLAYGROUND_MUJOCO_RENDER_PY_H

#include <pybind11/pybind11.h>
#include <mujoco_render/mujoco_render.h>

namespace py = pybind11;

void bind_mujoco_render (py::module& m){
    py::class_<mujoco_render::Render> RenderT(m, "Render");
    RenderT.def(py::init<mjModel*, mjData*, const std::string&, int, int, bool>(),
            py::arg("mjModel"),
            py::arg("mjData"),
            py::arg("window_name") = "Render"),
            py::arg("window_width") = 1200,
            py::arg("window_height") = 900;
}

#endif //MUJOCO_PLAYGROUND_MUJOCO_RENDER_PY_H
