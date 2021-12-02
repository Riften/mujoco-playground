//
// Created by yongxi on 2021/11/29.
//

#include <physics_mujoco/mujoco_kinematic_tree.h>
#include <physics_mujoco/exceptions.h>
#include <physics_mujoco/utils.h>
#include <log4cxx/logger.h>

static size_t MAX_LINKS = 1000;
static log4cxx::LoggerPtr logger = log4cxx::Logger::getLogger("KinematicTree");


namespace physics_mujoco {
    std::string kdl_vec_to_string(const KDL::Vector& vec) {
        char c_str[100];
        sprintf(c_str, "(%f, %f, %f)", vec.x(), vec.y(), vec.z());
        return std::string(c_str);
    }

    static KDL::Joint buildJoint(const mjModel * model, int joint_id, const KDL::Frame& body_frame) {
        // Fetch Joint Info
        const char* joint_name_ = mj_id2name(model, mjOBJ_JOINT, joint_id);
        if(joint_name_) {
            LOG4CXX_DEBUG(logger, "build joint " << joint_name_);
        }
        char joint_name[100];
        if(joint_name_) {
            sprintf(joint_name, "%s", joint_name_);
        } else {
            sprintf(joint_name, "unnamed_joint_%d", joint_id);
        }

        mjtJoint joint_type = static_cast<mjtJoint>(model->jnt_type[joint_id]);
        KDL::Joint joint;
        mjtNum joint_axis[3];
        mjtNum joint_origin[3];
        get_arr(model->jnt_axis, joint_id, 3, joint_axis);
        get_arr(model->jnt_pos, joint_id, 3, joint_origin);

        /**
        // Fetch body's translation and rotation
        // The axis of joint should rotate as the body rotation.
        // The origin of joint should move as the body translation.
        int body_id = model->jnt_bodyid[joint_id];
        mjtNum body_rot[4];
        mjtNum joint_axis_rotated[3];
        get_arr(model->body_quat, body_id, 4, body_rot);
        mju_rotVecQuat(joint_axis_rotated, joint_axis, body_rot);
         **/
        KDL::Vector kdl_joint_origin(joint_origin[0], joint_origin[1], joint_origin[2]);
        KDL::Vector kdl_joint_axis(joint_axis[0], joint_axis[1], joint_axis[2]);
        kdl_joint_axis = body_frame.M * kdl_joint_axis;
        kdl_joint_origin += body_frame.p;

        // body_frame.M.

        switch(joint_type) {
            case mjJNT_HINGE:
            case mjJNT_FREE:

                joint = KDL::Joint(joint_name,
                                 kdl_joint_origin,
                                 kdl_joint_axis,
                                 KDL::Joint::JointType::RotAxis);
                LOG4CXX_DEBUG(logger, "create joint with name(" << joint.getName()
                                                        << ") origin" << kdl_vec_to_string(joint.JointOrigin())
                                                        << " axis" << kdl_vec_to_string(joint.JointAxis())
                                                        << " type(" << joint.getTypeName() << ')');
                return joint;
                break;
            case mjJNT_BALL:
                throw UnsupportedJointType(joint_type);
                break;
            case mjJNT_SLIDE:

                joint = KDL::Joint(joint_name,
                                   kdl_joint_origin,
                                   kdl_joint_axis,
                                   KDL::Joint::JointType::TransAxis);
                LOG4CXX_DEBUG(logger, "create joint with name(" << joint.getName()
                                                                << ") origin" << kdl_vec_to_string(joint.JointOrigin())
                                                                << " axis" << kdl_vec_to_string(joint.JointAxis())
                                                                << " type(" << joint.getTypeName() << ')');
                return joint;

        }
    }

    bool KinematicTree::addLinkRecursive(const mjModel * model, int link_id, const std::string& hook_seg_name) {
        const char* link_name = mj_id2name(model, mjOBJ_BODY, link_id);
        if(link_name) {
            LOG4CXX_DEBUG(logger, "add link " << link_name << " to kdl tree");
        }

        // Fetch body frame
        KDL::Frame body_frame = mj_body_kdl_frame(model, link_id);

        std::string seg_name;
        if(link_name) {
            seg_name = link_name;
        } else {
            char tmp_name[100];
            sprintf(tmp_name, "noname_%d", link_id);
            seg_name = tmp_name;
        }


        /// @todo Load Inertia to KDL
        /// @todo Merge body / create dummy joint when joint number is 0
        // Build Joint
        int joint_num = model->body_jntnum[link_id];
        for(int i=0; i<joint_num; ++i) {
            int joint_id = model->body_jntadr[link_id] + i;
            KDL::Joint joint = buildJoint(model, joint_id, body_frame);
            KDL::Segment segment(seg_name, joint, body_frame);
            link_ids_[segment.getName()] = link_id;
            joint_ids_[joint.getName()] = joint_id;
            tree.addSegment(segment, hook_seg_name);
        }

        for(auto child : body_info_[link_id].children_id) {
            addLinkRecursive(model, child, seg_name);
        }

        return true;
    }

    KinematicTree::KinematicTree(const mjModel *model,
                                 const std::string &first_link,
                                 const std::string &last_link,
                                 const std::string& root_name)
    : tree(root_name)
    , model_(model){
        // Fetch body id
        int first_link_id = mj_name2id(model, mjOBJ_BODY, first_link.c_str());
        if(first_link_id < 0) {
            throw NoSuchLink(first_link.c_str());
        }
        int last_link_id = mj_name2id(model, mjOBJ_BODY, last_link.c_str());
        if(last_link_id < 0) {
            throw NoSuchLink(last_link.c_str());
        }

        // Fetch body pos
        body_info_ = fetch_body_info(model);
        /*
        LOG4CXX_DEBUG(logger, "Body Tree:");
        int current_id = first_link_id;
        do {
            LOG4CXX_DEBUG(logger, mj_id2name_safe());
        }while(current_id > 0);
         */
        addLinkRecursive(model, first_link_id, root_name);
    }

    void KinematicTree::print_kdl_tree() {
        TraverseKDLTree(tree.getRootSegment());
    }


}