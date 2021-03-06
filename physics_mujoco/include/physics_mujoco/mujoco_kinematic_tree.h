//
// Created by yongxi on 2021/11/29.
//

#ifndef MUJOCO_PLAYGROUND_MUJOCO_KINEMATIC_TREE_H
#define MUJOCO_PLAYGROUND_MUJOCO_KINEMATIC_TREE_H

#include <string>
#include <mujoco.h>
#include <kdl/tree.hpp>
#include <physics_mujoco/body_tree.h>

namespace physics_mujoco {
    class KinematicTree {
    public:
        /**
         * Build a KDL::Tree from mujoco mjModel.
         * @param model
         * @param first_link That is not the root link. In mujoco, static root link is not treated as a "link".
         * Instead, this is the link behind the first joint you want to plan for.
         * @param last_link The link behind the last joint you want to plan for.
         */
        KinematicTree(const mjModel *model,
                      const std::string& first_link,
                      const std::string& last_link,
                      const std::string& root_name = "kdl_root");
        void print_kdl_tree();
        KDL::Tree& kdl_tree() {return tree;}
        const mjModel* mujoco_model() const {return model_;}
        int mujoco_joint_id(const std::string& joint_name) const {
            return joint_ids_.at(joint_name);
        }
        int mujoco_link_id(const std::string& link_name) const {
            return link_ids_.at(link_name);
        }

    private:
        /**
         * The default constructor of KDL::Tree would create an empty tree with root_name = "root".
         */
        KDL::Tree tree;
        const mjModel * model_;
        std::vector<BodyInfo> body_info_;
        bool addLinkRecursive(const mjModel * model, int link_id, const std::string& hook_seg_name);
        std::map<std::string, int> joint_ids_; ///< Map from kdl joint name to mujoco joint index.
        std::map<std::string, int> link_ids_; ///< Map from kdl link/segment name to mujoco body index.
    };
}

#endif //MUJOCO_PLAYGROUND_MUJOCO_KINEMATIC_TREE_H
