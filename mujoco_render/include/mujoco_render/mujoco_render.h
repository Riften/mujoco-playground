//
// Created by yongxi on 2022/1/5.
//

#ifndef MUJOCO_PLAYGROUND_MUJOCO_RENDER_H
#define MUJOCO_PLAYGROUND_MUJOCO_RENDER_H
#include <mujoco.h>

namespace mujoco_render {
    class Render {
    public:
        Render(mjModel* model, mjData* data);
    private:
        mjModel* mj_model_;
        mjData* mj_data_;
    };
}

#endif //MUJOCO_PLAYGROUND_MUJOCO_RENDER_H
