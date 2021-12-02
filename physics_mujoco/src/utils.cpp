//
// Created by yongxi on 2021/11/29.
//

#include <physics_mujoco/utils.h>
#include <physics_mujoco/exceptions.h>
#include <mujoco.h>

namespace physics_mujoco {
    static const char* null_name = "NULL";

    void get_body_pos(const mjModel* model, int body_id, mjtNum * res) {
        memcpy(res, model->body_pos + body_id * 3, sizeof(mjtNum)*3);
    }

    void get_arr(const mjtNum* data, int index, int size, mjtNum * res) {
        memcpy(res, data + index * size, sizeof(mjtNum) * size);
    }

    const char* mj_id2name_err(const mjModel* m, int type, int id) {
        const char* res = mj_id2name(m, type, id);
        if(res) {
            return res;
        } else {
            throw NoSuchId(id);
        }
    }

    const char* mj_id2name_safe(const mjModel* m, int type, int id) {
        const char* name = mj_id2name(m, type, id);
        if(name){
            return name;
        } else {
            return null_name;
        }
    }

    std::string KDLFrameToString(const KDL::Frame& frame) {
        std::ostringstream sstr;
        double qx, qy, qz, qw;
        double ex, ey, ez;
        frame.M.GetQuaternion(qx, qy, qz, qw);
        frame.M.GetEulerZYX(ez, ey, ex);
        sstr << "{(" << frame.p.x() << ',' << frame.p.y() << ',' << frame.p.z() << "); "
             << '(' << ex << ',' << ey << ',' << ez << "); "
             << '(' << qx << ',' << qy << ',' << qz << ',' << qw << ")}";
        return sstr.str();
    }

    std::string KDLChainToString(const KDL::Chain& chain) {
        std::ostringstream sstr;
        sstr << '{';
        for(const auto& segment : chain.segments) {
            sstr << segment.getName() << '(' << segment.getJoint().getName() << "), ";
        }
        sstr << '}';
        return sstr.str();
    }

    std::string KDLJntArrayToString(const KDL::JntArray& joint_array) {
        std::ostringstream sstr;
        sstr << "[ ";
        for(size_t i=0; i<joint_array.data.size(); ++i) {
            sstr << joint_array(i) << ", ";
        }
        sstr <<']';
        return sstr.str();
    }

    std::string KDLVectorToString(const KDL::Vector& vec) {
        std::ostringstream sstr;
        sstr << "[ " << vec.x() << ", " << vec.y() << ", " << vec.z() << "]";
        return sstr.str();
    }

    std::string EigenAffine3dToString(const Eigen::Affine3d& transform) {
        std::ostringstream sstr;

    }

    KDL::Frame mj_body_kdl_frame(const mjModel *m, int body_id) {
        mjtNum body_pos[3];
        mjtNum body_quat[4];
        get_body_pos(m, body_id, body_pos);
        get_arr(m->body_quat, body_id, 4, body_quat);
        return {KDL::Rotation::Quaternion(body_quat[1], body_quat[2], body_quat[3], body_quat[0]),
                KDL::Vector(body_pos[0], body_pos[1], body_pos[2])};
    }

    KDL::Frame mj_body_kdl_xframe(const mjData *data, int body_id) {
        mjtNum body_pos[3];
        mjtNum body_quat[4];
        get_arr(data->xpos, body_id, 3, body_pos);
        get_arr(data->xquat, body_id, 4, body_quat);
        return {KDL::Rotation::Quaternion(body_quat[1], body_quat[2], body_quat[3], body_quat[0]),
                KDL::Vector(body_pos[0], body_pos[1], body_pos[2])};
    }

    void TraverseKDLTree(KDL::SegmentMap::const_iterator node, int indent) {
        for(int i=0; i<indent; ++i) {
            std::cout << "  ";
        }
        std::cout << node->first << "  " << KDLFrameToString(node->second.segment.getFrameToTip())
                  << " " << node->second.segment.getJoint().getName() << KDLVectorToString(node->second.segment.getJoint().JointOrigin())
                  << KDLVectorToString(node->second.segment.getJoint().JointAxis())<<std::endl;

        for(auto child : node->second.children) {
            TraverseKDLTree(child, indent+1);
        }
    }
}