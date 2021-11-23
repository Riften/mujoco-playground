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

    class InsufficientContainer : public MujocoException {
    public:
        InsufficientContainer(size_t given, size_t required) {
            sprintf(msg_, "container too small: %zu given while %zu required", given, required);
        }
    };

    class UnsupportedControlMode : public MujocoException {
    public:
        UnsupportedControlMode() {
            sprintf(msg_, "unsupported control mode");
        }
    };
}

#endif //MUJOCO_PLAYGROUND_EXCEPTIONS_H
