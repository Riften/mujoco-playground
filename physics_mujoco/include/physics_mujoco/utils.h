//
// Created by yongxi on 2021/11/24.
//

#ifndef MUJOCO_PLAYGROUND_UTILS_H
#define MUJOCO_PLAYGROUND_UTILS_H

#include <map>
#include <vector>
#include <cstring>
#include <mjmodel.h>
#include <exception>
#include <log4cxx/propertyconfigurator.h>
#include <log4cxx/basicconfigurator.h>
#include <fstream>
#include <iostream>

namespace physics_mujoco {

    static inline bool file_exists(const char* file_path) {
        std::ifstream f(file_path);
        return f.good();
    }

    void inline initLogSystem() {
        char DEFAULT_LOG4CXX_CONFIG_PATH[200];
        sprintf(DEFAULT_LOG4CXX_CONFIG_PATH, "%s/.rfpddl.config", std::getenv("HOME"));
        if(file_exists(DEFAULT_LOG4CXX_CONFIG_PATH)) {
            std::cout << "... Initialize log system from " << DEFAULT_LOG4CXX_CONFIG_PATH << std::endl;
            log4cxx::PropertyConfigurator::configure(DEFAULT_LOG4CXX_CONFIG_PATH);
        } else {
            std::cout << "... Initialize default log system." << std::endl;
            log4cxx::BasicConfigurator::configure();
        }
    }

    /**
     * Convert array to map for quick index/search.
     * @tparam T type of array element which is also the type of map key.
     */
    template <typename T>
    class array_map {
    private:
        std::map<T, size_t> map_;
        size_t size_;
    public:
        array_map() {
            size_ = 0;
        };
        array_map(T* arr, size_t n) {
            size_ = n;
            for(size_t i=0; i<n; ++i) {
                map_[arr[i]] = i;
            }
        }

        explicit array_map(std::vector<T> arr) {
            size_ = arr.size();
            for(size_t i=0; i < size_; ++i) {
                map_[arr[i]] = i;
            }
        }

        size_t size() const {
            return size_;
        }

        bool has(T var) const {
            return map_.find(var) != map_.cend();
        }

        size_t index(T var) const {
            return map_[var];
        }
    };

    template <typename TKey, typename Tp>
    bool keyInMap(const std::map<TKey, Tp>& data, const TKey& key) {
        return data.find(key) != data.cend();
    }

    /**
     * Get mjModel::body_pos[body_id] and copy it in res.
     * @param model
     * @param body_id
     * @param res
     */
    void get_body_pos(const mjModel* model, int body_id, mjtNum * res);

    void get_arr(const mjtNum* data, int index, int size, mjtNum * res);

    const char* mj_id2name_err(const mjModel* m, int type, int id);
    const char* mj_id2name_safe(const mjModel* m, int type, int id);
}

#endif //MUJOCO_PLAYGROUND_UTILS_H
