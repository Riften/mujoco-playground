//
// Created by yongxi on 2021/11/21.
//

#ifndef MUJOCO_PLAYGROUND_MUJOCO_JOINT_GROUP_H
#define MUJOCO_PLAYGROUND_MUJOCO_JOINT_GROUP_H

#include <physics_interface/joint_group.h>
#include <physics_mujoco/exceptions.h>
#include <string>
#include <mujoco.h>
#include <physics_mujoco/utils.h>
#include <physics_mujoco/mujoco_kinematic_tree.h>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
// #include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp> // jl means joint limits
#include <kdl/chainiksolvervel_pinv.hpp>
// #include <kdl/chainiksolverpos_lma.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

/// OMPL
#include <ompl/geometric/SimpleSetup.h>

namespace physics_mujoco {
    // typedef  Eigen::Transform<mjtNum, 3, Eigen::Affine> Affine3d;

    class JointGroup: public physics_interface::JointGroup{
    public:
        JointGroup(mjData * data, KinematicTree &tree,
                   const std::string& start_link_name,
                   const std::string& end_link_name);
        ~JointGroup() override;
        void getPos(std::vector<physics_interface::JointPos>& res) override;
        void getPos(KDL::JntArray& res);
        void getVel(std::vector<physics_interface::JointVel>& res) override;
        void setPos(KDL::JntArray & jnt_pos);
        KDL::JntArray setRandom();
        KDL::Frame eefPos();
        KDL::Frame FK();
        /**
         * @todo IK should return an error code if there is no solution.
         * @param tip_pos
         * @return
         */
        KDL::JntArray IK(const KDL::Frame& tip_pos);
        size_t size() const {return n_jnt_;}
        double lowerBound(int i) const {return kdl_jnt_min_(i);};
        double upperBound(int i) const {return kdl_jnt_max_(i);};
        bool isLimited(int i) const { return is_limited_[i];}

        /**
         * Whether links in this group is in collision with any geoms.
         * @return
         */
        bool inCollision();

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

        /// OMPL
        bool motionPlan(const KDL::JntArray& start, const KDL::JntArray& end, ompl::geometric::PathGeometric& res);
        bool motionPlan(const KDL::JntArray& start, const KDL::JntArray& end);
        ompl::geometric::PathGeometric& currentSolution();


    private:
        const mjModel * model_;
        mjData * data_;
        std::vector<int> qpos_indices_;
        std::vector<int> qvel_indices_;
        std::vector<int> link_ids_;
        std::vector<int> joint_ids_;
        std::vector<bool> is_limited_;
        std::vector<physics_interface::JointPos> target_pos_;
        // std::vector<physics_interface::JointPos> target_pos_inc_;
        size_t n_jnt_;
        std::map<int, int> geom_body_map_;

        void control_incremental_impedance();

        /// Impedance control parameters
        double kp_ = 1;
        double kd_;
        double damp_ratio_ = 1;
        double max_acc_ = 5;

        mjtNum* tmp_res_;

        void vel_to_tmp();

        /// OROCOS KDL
        KDL::Chain chain_;
        // KDL::JntArray kdl_jnt_pos_;
        KDL::ChainFkSolverPos_recursive* kdl_fk_solver_ = nullptr; // solver have no default constructor
        KDL::ChainIkSolverPos_NR_JL* kdl_ik_solver_ = nullptr;
        KDL::ChainIkSolverVel_pinv* kdl_ik_vel_solver_ = nullptr;
        KDL::JntArray kdl_jnt_min_;
        KDL::JntArray kdl_jnt_max_;
        KDL::JntArray kdl_jnt_default_;

        /// OMPL
        ompl::geometric::SimpleSetupPtr simple_setup_;
        // ompl::base::SpaceInformationPtr state_spece_;
        void _init_ompl();
    };
}

#endif //MUJOCO_PLAYGROUND_MUJOCO_JOINT_GROUP_H
