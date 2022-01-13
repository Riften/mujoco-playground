//
// Created by yongxi on 2022/1/12.
//

#ifndef MUJOCO_PLAYGROUND_SIMULATE_THREAD_H
#define MUJOCO_PLAYGROUND_SIMULATE_THREAD_H

#include <thread>
#include <mujoco.h>
#include <mutex>
#include <atomic>
#include <log4cxx/logger.h>

/**
 * @todo We need an atomic struct for simulate control.
 */
class SimulateThread {
public:
    SimulateThread(mjModel * model, mjData * data, std::mutex* mtx)
    : model_(model)
    , data_(data)
    , mtx_(mtx)
    , exit_(false){
        logger = log4cxx::Logger::getLogger("SimulateThread");
        thread_ = new std::thread([this](){
            this->simulate_loop();
        });
    }

    ~SimulateThread() {
        exit_ = true;
        thread_->join();
        delete thread_;
    }
private:
    mjModel * model_;
    mjData * data_;
    std::mutex* mtx_;
    std::thread* thread_;
    std::atomic<bool> exit_;
    log4cxx::LoggerPtr logger;
    void simulate_loop() {
        LOG4CXX_DEBUG(logger, "Simulate loop starts");
        while(!exit_) {
            mtx_->lock();
            mj_step(model_, data_);
            mtx_->unlock();
        }
        LOG4CXX_DEBUG(logger, "Simulate loop ends");
    }
};

#endif //MUJOCO_PLAYGROUND_SIMULATE_THREAD_H
