//
// Created by yongxi on 2021/11/21.
//

#include <physics_mujoco/mujoco_joint_group.h>

namespace physics_mujoco {

    static JointGroup* control_group = nullptr;
    void control_callback_incremental(const mjModel* model, mjData * data) {
        if(control_group != nullptr) {
            control_group->control_incremental_impedance();
        }
    }

    JointGroup::JointGroup(mjModel *model, mjData *data, const std::vector<std::string> &joint_names)
    : model_(model)
    , data_(data)
    , qpos_indices_(joint_names.size())
    , qvel_indices_(joint_names.size())
    , n_jnt_(joint_names.size())
    , kd_(0){
        // Fetch indices
        for(int i=0; i<joint_names.size(); ++i) {
            int id = mj_name2id(model_, mjOBJ_JOINT, joint_names[i].c_str());
            if (id < 0) {
                throw NoSuchJoint(joint_names[i].c_str());
            }
            qpos_indices_[i] = model_->jnt_qposadr[id];
            qvel_indices_[i] = model_->jnt_dofadr[id];
        }

        // mem alloc for tmp_res_
        /// @todo Array size may be insufficient if one joint have multi velocity
        tmp_res_ = (mjtNum*) mju_malloc(sizeof(mjtNum) * n_jnt_);
    }

    JointGroup::~JointGroup() {
        mju_free(tmp_res_);
    }

    void JointGroup::getPos(std::vector<physics_interface::JointPos> &res) {
        if(res.size() < n_jnt_) {
            throw InsufficientContainer(res.size(), n_jnt_);
        }
        for(int i=0; i< n_jnt_; ++i) {
            /// @todo multi joint support
            res[i] = data_->qpos[qpos_indices_[i]];
        }
    }

    void JointGroup::getVel(std::vector<physics_interface::JointVel> &res) {
        if(res.size() < n_jnt_) {
            throw InsufficientContainer(res.size(), n_jnt_);
        }
        for(int i=0; i< n_jnt_; ++i) {
            /// @todo multi joint support
            res[i] = data_->qvel[qvel_indices_[i]];
        }
    }

    void JointGroup::control(const std::vector<physics_interface::JointPos> &pos, physics_interface::ControlType type) {
        switch (type) {
            case physics_interface::IMPEDANCE:

                break;
            default:
                throw UnsupportedControlMode();
        }
    }

    void JointGroup::control_incremental(const std::vector<physics_interface::JointPos> &pos_inc,
                                         physics_interface::ControlType type) {
        switch (type) {
            case physics_interface::IMPEDANCE:
                target_pos_ = pos_inc;
                kd_ = 2*mju_sqrt(kp_) * damp_ratio_;
                control_group = this;
                mjcb_control = control_callback_incremental;
                break;
            default:
                throw UnsupportedControlMode();
        }
    }

    void JointGroup::vel_to_tmp() {
        for(int i=0; i < n_jnt_; ++i) {
            tmp_res_[i] = data_->qvel[qvel_indices_[i]];
        }
    }



    void JointGroup::control_incremental_impedance() {
        vel_to_tmp();
        ///=====================================
    }


}