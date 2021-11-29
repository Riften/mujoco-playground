//
// Created by yongxi on 2021/11/30.
//

#include <physics_mujoco/body_tree.h>

namespace physics_mujoco {
    std::vector<BodyInfo> fetch_body_info(const mjModel* model) {
        std::vector<BodyInfo> res(model->nbody);
        for(int i=0; i<model->nbody; ++i) {
            res[i].id = i;
            res[i].parent_id = model->body_parentid[i];
            res[model->body_parentid[i]].children_id.push_back(i);
        }
        return res;
    }
}