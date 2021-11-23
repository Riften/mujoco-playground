//
// Created by yongxi on 2021/11/21.
//

#include <physics_mujoco/mujoco_joint_controller.h>
#include <physics_mujoco/exceptions.h>
#include <iostream>

namespace physics_mujoco {
    JointController::JointController(mjModel *model, mjData *data, const char *joint_name,
                                     double default_pos, double default_vel) {
        model_ = model;
        data_ = data;
        target_pos_ = default_pos;
        target_vel_ = default_vel;
        joint_index_ = mj_name2id(model, mjOBJ_JOINT, joint_name);
        if(joint_index_ < 0) {
            throw NoSuchJoint(joint_name);
        }

        kp_ = 5;
        damp_ratio_ = 1;
        kd_ = 2 * sqrt(kp_) * damp_ratio_;



        max_acc_ = 2;
    }

    JointController::~JointController() {
    }

    void JointController::impedance_pos_control(double target_pos) {
        double position_error = target_pos - getPos();
        while(position_error > M_PI) {
            position_error -= 2*M_PI;
        }
        while(position_error < -M_PI) {
            position_error += 2*M_PI;
        }

        // Compute the joint space inertia on this joint
        mjtNum * index_vec_ = (mjtNum*) mju_malloc(sizeof(mjtNum) * model_->nv);
        mjtNum * res_nv_ = (mjtNum*) mju_malloc(sizeof(mjtNum) * model_->nv);
        mju_zero(index_vec_, model_->nv);
        index_vec_[joint_index_] = max_acc_;
        mj_mulM(model_, data_, res_nv_, index_vec_);
        double max_torque = mju_abs(res_nv_[joint_index_]);
        double torque = position_error * kp_ - getVel() * kd_ + data_->qfrc_bias[joint_index_];

        // double torque = data_->qfrc_bias[joint_index_];

        if(torque > max_torque) {
            torque = max_torque;
        }
        if(torque < -max_torque) {
            torque = -max_torque;
        }


        // double torque = data_->qfrc_bias[joint_index_];
        // std::cout << "Index: " << joint_index_ << " Error: " <<  position_error << " M" << res_nv_[joint_index_] << " Torque: " << torque << std::endl;
        data_->qfrc_applied[joint_index_] = torque;
        mju_free(index_vec_);
        mju_free(res_nv_);
    }

    void JointController::setPos(double pos) {
        target_pos_ = pos;
        impedance_pos_control(pos);
    }

    double JointController::getPos() {
        return data_->qpos[joint_index_];
    }

    double JointController::getVel() {
        return data_->qvel[joint_index_];
    }
}