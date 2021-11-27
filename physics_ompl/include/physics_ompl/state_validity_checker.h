//
// Created by yongxi on 2021/11/25.
//

#ifndef MUJOCO_PLAYGROUND_STATE_VALIDITY_CHECKER_H
#define MUJOCO_PLAYGROUND_STATE_VALIDITY_CHECKER_H

#include <ompl/base/StateValidityChecker.h>

namespace physics_ompl {
    class StateValidityChecker: ompl::base::StateValidityChecker {
    public:
        StateValidityChecker(const ompl::base::SpaceInformationPtr &si);
    };
}

#endif //MUJOCO_PLAYGROUND_STATE_VALIDITY_CHECKER_H
