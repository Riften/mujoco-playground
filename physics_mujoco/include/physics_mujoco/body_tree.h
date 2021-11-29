//
// Created by yongxi on 2021/11/30.
//

#ifndef MUJOCO_PLAYGROUND_BODY_TREE_H
#define MUJOCO_PLAYGROUND_BODY_TREE_H

#include <vector>
#include <mjmodel.h>

namespace physics_mujoco {
    struct BodyInfo {
        int id;
        int parent_id;
        std::vector<int> children_id;
    };

    /**
     * Fetch all body info into a BodyInfo vector.
     *
     * @details MuJoCo MJCF script build kinematic tree for bodies.
     * However, in mjModel, only 'parent body' index can be fetched for a specific body.
     * That makes it difficult to build kinematic tree from mjModel directly.
     *
     * @param model
     * @return
     */
    std::vector<BodyInfo> fetch_body_info(const mjModel* model);
}

#endif //MUJOCO_PLAYGROUND_BODY_TREE_H
