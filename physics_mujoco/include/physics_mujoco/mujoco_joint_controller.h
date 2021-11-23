//
// Created by yongxi on 2021/11/21.
//

#ifndef MUJOCO_PLAYGROUND_MUJOCO_JOINT_CONTROLLER_H
#define MUJOCO_PLAYGROUND_MUJOCO_JOINT_CONTROLLER_H

#include <physics_interface/joint_controller.h>
#include <mujoco.h>

namespace physics_mujoco {
    class JointController: public physics_interface::JointController {
    public:
        JointController(mjModel* model, mjData* data, const char* joint_name,
                        double default_pos = 0, double default_vel = 0);
        ~JointController() override;
        double getPos() override;
        double getVel() override;
        void setPos(double pos) override;
        void impedance_pos_control(double target_pos);

    private:
        double target_pos_;
        double target_vel_;
        mjModel * model_;
        mjData * data_;
        int joint_index_;

        // Impedance parameter
        double kp_;
        double damp_ratio_;
        double kd_;
        double max_acc_;
        // mjtNum * index_vec_ = nullptr; // v = [0,0,...,1,0,0 ...] where v[joint_index_] = 1
        // mjtNum * res_nv_ = nullptr; // an array with size (nv x 1) to hold the computation result.
    };
}

#endif //MUJOCO_PLAYGROUND_MUJOCO_JOINT_CONTROLLER_H
