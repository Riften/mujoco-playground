//
// Created by yongxi on 2021/11/25.
//

#include <physics_ompl/state_validity_checker.h>

namespace physics_ompl {
    StateValidityChecker::StateValidityChecker(const ompl::base::SpaceInformationPtr &si):
            ompl::base::StateValidityChecker(si){

    }
}