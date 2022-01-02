//
// Created by yongxi on 2021/12/3.
//

#ifndef MUJOCO_PLAYGROUND_MUJOCO_STATE_VALIDITY_CHACKER_H
#define MUJOCO_PLAYGROUND_MUJOCO_STATE_VALIDITY_CHACKER_H

#include <physics_mujoco/mujoco_joint_group.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
namespace physics_mujoco {
    class StateValidityChecker : public ompl::base::StateValidityChecker {
    public:
        StateValidityChecker(JointGroup* jointGroup, const ompl::base::SpaceInformationPtr& si)
                : ompl::base::StateValidityChecker(si)
                , joint_group_(jointGroup){

        }

        bool isValid(const ompl::base::State* state) const override {
            auto r_state = state->as<ompl::base::RealVectorStateSpace::StateType>();
            KDL::JntArray q(joint_group_->size());
            for(int i=0; i<joint_group_->size(); ++i) {
                q(i) = (*r_state)[i];
            }
            joint_group_->setPos(q);
            return !joint_group_->inCollision();
        }

    private:
        /// We use raw pointer directly because checker is created within JointGroup most time.
        JointGroup* joint_group_;
    };
}

#endif //MUJOCO_PLAYGROUND_MUJOCO_STATE_VALIDITY_CHACKER_H
