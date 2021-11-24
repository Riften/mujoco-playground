//
// Created by yongxi on 2021/11/21.
//

#include <physics_mujoco/mujoco_joint_group.h>
#include <log4cxx/logger.h>

static log4cxx::LoggerPtr logger = log4cxx::Logger::getLogger("JointGroup");

namespace physics_mujoco {

    static JointGroup* control_group = nullptr;
    void control_callback_incremental(const mjModel* model, mjData * data) {
        if(control_group != nullptr) {
            control_group->control_incremental_impedance();
        }
    }

    JointGroup::JointGroup(mjModel *model, mjData *data,
                           const std::vector<std::string> &joint_names,
                           const std::vector<std::string> &link_names)
    : model_(model)
    , data_(data)
    , qpos_indices_(joint_names.size())
    , qvel_indices_(joint_names.size())
    , n_jnt_(joint_names.size())
    , kd_(0)
    , link_ids_(link_names.size()) {
        // Fetch indices
        for(int i=0; i<joint_names.size(); ++i) {
            int id = mj_name2id(model_, mjOBJ_JOINT, joint_names[i].c_str());
            if (id < 0) {
                throw NoSuchJoint(joint_names[i].c_str());
            }
            qpos_indices_[i] = model_->jnt_qposadr[id];
            qvel_indices_[i] = model_->jnt_dofadr[id];
        }

        for(int i=0; i<link_names.size(); ++i) {
            link_ids_[i] = mj_name2id(model_, mjOBJ_BODY, link_names[i].c_str());
            if (link_ids_[i] < 0) {
                throw NoSuchJoint(link_names[i].c_str());
            }
        }

        // mem alloc for tmp_res_
        /// @todo Array size may be insufficient if one joint have multi velocity
        tmp_res_ = (mjtNum*) mju_malloc(sizeof(mjtNum) * n_jnt_);

        array_map<int> link_id_map(link_ids_);
        for(int i=0; i < model_->ngeom; ++i) {
            if(link_id_map.has(model_->geom_bodyid[i])) {
                geom_body_map_[i] = model_->geom_bodyid[i];
            }
        }
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

    bool JointGroup::inCollision() {
        // Check all contacts
        for(int i=0; i<data_->ncon; ++i) {
            if(keyInMap(geom_body_map_, data_->contact[i].geom1)
            || keyInMap(geom_body_map_, data_->contact[i].geom2)) {
                return true;
            }
        }
        return false;
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