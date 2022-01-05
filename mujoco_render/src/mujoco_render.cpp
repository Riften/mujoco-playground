//
// Created by yongxi on 2022/1/5.
//

#include <mujoco_render/mujoco_render.h>

namespace mujoco_render {
    Render::Render(mjModel *model, mjData *data)
    : mj_model_(model)
    , mj_data_(data){

    }
}