//
// Created by yongxi on 2021/11/21.
//

#ifndef MUJOCO_PLAYGROUND_JOINT_GROUP_H
#define MUJOCO_PLAYGROUND_JOINT_GROUP_H

#include <vector>

namespace physics_interface {
    /// @todo: Some Joint may contains multi values. Design a class for JointPos.
    /// @note: JointPos should be implemented in interface lib and shared by all engines.
    typedef double JointPos;
    typedef double JointVel;

    enum ControlType{
        IMPEDANCE = 0
    };

    class JointGroup {
    public:
        virtual ~JointGroup() = default;
        virtual void getPos(std::vector<JointPos>& res) = 0;
        virtual void getVel(std::vector<JointVel>& res) = 0;
        /// @todo add callback or timeout argument
        virtual void control(const std::vector<JointPos>& pos, ControlType type = IMPEDANCE) = 0;
        virtual void control_incremental(const std::vector<JointPos>& pos_inc, ControlType type = IMPEDANCE) = 0;
    };
}

#endif //MUJOCO_PLAYGROUND_JOINT_GROUP_H
