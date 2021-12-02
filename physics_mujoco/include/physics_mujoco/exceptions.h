//
// Created by yongxi on 2021/11/21.
//

#ifndef MUJOCO_PLAYGROUND_EXCEPTIONS_H
#define MUJOCO_PLAYGROUND_EXCEPTIONS_H

#include <exception>
#include <cstdio>

namespace physics_mujoco {
    class MujocoException: public std::exception{
    public:
        char *msg_;
        MujocoException() {
            msg_ = new char[1000];
        }

        const char* what() const noexcept override {
            return msg_;
        }

        ~MujocoException() override {
            delete msg_;
        }

    };

    class NoSuchJoint : public MujocoException {
    public:
        explicit NoSuchJoint(const char* joint_name):MujocoException() {
            sprintf(msg_, "no such joint %s", joint_name);
        }
    };

    class NoSuchLink : public MujocoException {
    public:
        explicit NoSuchLink(const char* link_name): MujocoException() {
            sprintf(msg_, "no such link %s", link_name);
        }
     };

    class NoSuchName : public MujocoException {
    public:
        explicit NoSuchName(const char* name) : MujocoException() {
            sprintf(msg_, "no such name %s", name);
        }
    };

    class NoSuchId : public MujocoException {
    public:
        explicit NoSuchId(int id) : MujocoException() {
            sprintf(msg_, "no name for %d", id);
        }
    };

    class InsufficientContainer : public MujocoException {
    public:
        InsufficientContainer(size_t given, size_t required) : MujocoException() {
            sprintf(msg_, "container too small: %zu given while %zu required", given, required);
        }
    };

    class UnsupportedControlMode : public MujocoException {
    public:
        UnsupportedControlMode() : MujocoException() {
            sprintf(msg_, "unsupported control mode");
        }
    };

    class UnsupportedJointType : public MujocoException {
    public:
        explicit UnsupportedJointType(int joint_type) : MujocoException() {
            sprintf(msg_, "unsupported joint type %d", joint_type);
        }
    };

    class FailToCreateChain : public MujocoException {
    public:
        FailToCreateChain(const char* start, const char* end): MujocoException() {
            sprintf(msg_, "create kdl kinematic chain from %s to %s failed", start, end);
        }
    };
}

#endif //MUJOCO_PLAYGROUND_EXCEPTIONS_H
