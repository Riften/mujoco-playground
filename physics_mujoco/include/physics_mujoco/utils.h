//
// Created by yongxi on 2021/11/24.
//

#ifndef MUJOCO_PLAYGROUND_UTILS_H
#define MUJOCO_PLAYGROUND_UTILS_H

#include <map>
#include <vector>

namespace physics_mujoco {
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
}

#endif //MUJOCO_PLAYGROUND_UTILS_H
