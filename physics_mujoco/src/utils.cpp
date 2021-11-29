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
}