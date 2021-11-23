//
// Created by yongxi on 2021/11/21.
//

#ifndef MUJOCO_PLAYGROUND_MUJOCO_JOINT_GROUP_H
#define MUJOCO_PLAYGROUND_MUJOCO_JOINT_GROUP_H

#include <physics_interface/joint_group.h>
#include <physics_mujoco/exceptions.h>
#include <string>
#include <mujoco.h>

namespace physics_mujoco {

    class JointGroup: public physics_interface::JointGroup{
    public:
        /// @todo mark model as const if wo do not change the property of model directly.
        JointGroup(mjModel * model, mjData * data, const std::vector<std::string>& joint_names);
        ~JointGroup() override;
        void getPos(std::vector<physics_interface::JointPos>& res) override;
        void getVel(std::vector<physics_interface::JointVel>& res) override;

        /**
         * @note It is not a good idea to use default value in virtual function.
         * Make sure that they are the same in all inherited classes.
         *
         * @param pos
         * @param type
         */
        void control(const std::vector<physics_interface::JointPos>& pos,
                     physics_interface::ControlType type = physics_interface::IMPEDANCE) override;

        void control_incremental(const std::vector<physics_interface::JointPos>& pos_inc,
                                 physics_interface::ControlType type = physics_interface::IMPEDANCE) override;


        // mjModel * mj_model() {return model_;}
        // mjData * mj_data() {return data_;}
        friend void control_callback_incremental(const mjModel* model, mjData * data);
    private:
        mjModel * model_;
        mjData * data_;
        std::vector<int> qpos_indices_;
        std::vector<int> qvel_indices_;
        std::vector<physics_interface::JointPos> target_pos_;
        // std::vector<physics_interface::JointPos> target_pos_inc_;
        size_t n_jnt_;

        void control_incremental_impedance();

        /// Impedance control parameters
        double kp_ = 1;
        double kd_;
        double damp_ratio_ = 1;
        double max_acc_ = 5;

        mjtNum* tmp_res_;

        void vel_to_tmp();
    };
}

#endif //MUJOCO_PLAYGROUND_MUJOCO_JOINT_GROUP_H
