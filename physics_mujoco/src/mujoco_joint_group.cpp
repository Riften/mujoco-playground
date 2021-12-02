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

    JointGroup::JointGroup(mjData *data, KinematicTree &tree, const std::string &start_link_name,
                           const std::string &end_link_name)
    : model_(tree.mujoco_model())
    , data_(data)
    , kd_(0){
        // Create KDL Chain
        if(!tree.kdl_tree().getChain(start_link_name, end_link_name, chain_)) {
            LOG4CXX_ERROR(logger, "create KDL Chain from "
                               << start_link_name << " to "
                               << end_link_name << "failed");
            throw FailToCreateChain(start_link_name.c_str(), end_link_name.c_str());
        }
        LOG4CXX_DEBUG(logger, "Create chain: " << KDLChainToString(chain_));

        // Fetch Link and Joint from chain
        n_jnt_ = chain_.getNrOfSegments();
        qpos_indices_.resize(n_jnt_);
        qvel_indices_.resize(n_jnt_);
        link_ids_.resize(n_jnt_);
        joint_ids_.resize(n_jnt_);
        kdl_jnt_max_.resize(n_jnt_);
        kdl_jnt_min_.resize(n_jnt_);
        kdl_jnt_default_.resize(n_jnt_);
        SetToZero(kdl_jnt_default_);
        for(int i=0; i<chain_.getNrOfSegments(); ++i) {
            link_ids_[i] = tree.mujoco_link_id(chain_.getSegment(i).getName());
            int joint_id = tree.mujoco_joint_id(chain_.getSegment(i).getJoint().getName());
            joint_ids_[i] = joint_id;
            qpos_indices_[i] = model_->jnt_qposadr[joint_id];
            qvel_indices_[i] = model_->jnt_dofadr[joint_id];
            if(model_->jnt_limited[i]) {
                kdl_jnt_min_(i) = model_->jnt_range[i*2];
                kdl_jnt_max_(i) = model_->jnt_range[i*2+1];
            } else {
                kdl_jnt_min_(i) = -M_PI;
                kdl_jnt_max_(i) = M_PI;
            }
        }
        std::ostringstream tmp_sstr;
        tmp_sstr << '[';
        for(int i=0; i<chain_.getNrOfSegments(); ++i) {
            tmp_sstr << qpos_indices_[i] << ',';
        }
        tmp_sstr << ']';
        LOG4CXX_DEBUG(logger, "qpos_indices_:" << tmp_sstr.str());

        // mem alloc for tmp_res_
        /// @todo Array size may be insufficient if one joint have multi velocity
        tmp_res_ = (mjtNum*) mju_malloc(sizeof(mjtNum) * n_jnt_);

        array_map<int> link_id_map(link_ids_);
        for(int i=0; i < model_->ngeom; ++i) {
            if(link_id_map.has(model_->geom_bodyid[i])) {
                geom_body_map_[i] = model_->geom_bodyid[i];
            }
        }

        // Fetch joint limit info


        // create kdl solver
        kdl_fk_solver_ = new KDL::ChainFkSolverPos_recursive(chain_);
        kdl_ik_vel_solver_ = new KDL::ChainIkSolverVel_pinv(chain_);
        kdl_ik_solver_ = new KDL::ChainIkSolverPos_NR_JL(chain_,
                                                         kdl_jnt_min_,
                                                         kdl_jnt_max_,
                                                         *kdl_fk_solver_,
                                                         *kdl_ik_vel_solver_,
                                                         100,
                                                         1e-6);
    }
/**
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
    **/

    JointGroup::~JointGroup() {
        mju_free(tmp_res_);
        delete kdl_fk_solver_;
        delete kdl_ik_vel_solver_;
        delete kdl_ik_solver_;
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

    void JointGroup::setPos(KDL::JntArray &jnt_pos) {
        if(jnt_pos.data.size() < n_jnt_) {
            LOG4CXX_ERROR(logger, "Not enough jnt pos when set pos: "
                                   << jnt_pos.data.size() << " given while " << n_jnt_ << " required.");
            return;
        }
        for(int i=0; i< n_jnt_; ++i) {
            data_->qpos[qpos_indices_[i]] = jnt_pos(i);
        }
        mj_step1(model_, data_);
    }

    KDL::Frame JointGroup::eefPos() {
        int eef_id = link_ids_[link_ids_.size()-1];
        LOG4CXX_DEBUG(logger, "get end effector pos for " << mj_id2name_err(model_, mjOBJ_BODY, eef_id));
        return mj_body_kdl_xframe(data_, link_ids_[link_ids_.size()-1]);
    }

    KDL::Frame JointGroup::FK() {
        // Fetch joint position into kdl_jnt_pos_
        KDL::JntArray kdl_jnt_pos(n_jnt_);
        for(int i = 0; i<n_jnt_; ++i) {
            kdl_jnt_pos(i) = data_->qpos[qpos_indices_[i]];
        }
        LOG4CXX_DEBUG(logger, "Compute Forward Kinematics with joint pos: " << KDLJntArrayToString(kdl_jnt_pos));

        KDL::Frame res;
        kdl_fk_solver_->JntToCart(kdl_jnt_pos, res);
        LOG4CXX_DEBUG(logger, "FK res: [ " << res.p[0] << ", " << res.p[1] << ", " << res.p[2] << ']');
        return res;
    }

    KDL::JntArray JointGroup::IK(const KDL::Frame& tip_pos) {
        KDL::JntArray res(n_jnt_);
        LOG4CXX_DEBUG(logger, "Compute IK for tip: " << KDLFrameToString(tip_pos));
        kdl_ik_solver_->CartToJnt(kdl_jnt_default_, tip_pos, res);
        return res;
    }

    bool JointGroup::inCollision() {
        // Check all contacts
        /**
        std::cout << "Contacts: ";
        for(int i=0; i < data_->ncon; ++i) {
            std::cout << '('
                      << mj_id2name(model_, mjOBJ_BODY, model_->geom_bodyid[data_->contact[i].geom1])
                      << ','
                      << mj_id2name(model_, mjOBJ_BODY, model_->geom_bodyid[data_->contact[i].geom2])
                      << ") ";
        }
        std::cout << std::endl;
         **/

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