//
// Created by yongxi on 2021/11/21.
//

#ifndef MUJOCO_PLAYGROUND_JOINT_CONTROLLER_H
#define MUJOCO_PLAYGROUND_JOINT_CONTROLLER_H

namespace physics_interface {
    class JointController {
    public:
        virtual void setPos(double pos) = 0;
        virtual double getPos() = 0;
        virtual double getVel() = 0;
        virtual ~JointController()= default;
    };
}

#endif //MUJOCO_PLAYGROUND_JOINT_CONTROLLER_H
