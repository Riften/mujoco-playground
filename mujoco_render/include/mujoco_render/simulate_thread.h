//
// Created by yongxi on 2022/1/12.
//

#ifndef MUJOCO_PLAYGROUND_SIMULATE_THREAD_H
#define MUJOCO_PLAYGROUND_SIMULATE_THREAD_H

#include <thread>
#include <mujoco.h>
#include <mutex>
#include <atomic>
#include <utility>
#include <log4cxx/logger.h>

/**
 * @todo We need an atomic struct for simulate control.
 */
class SimulateThread {
public:
    SimulateThread(mjModel * model, mjData * data,
                   std::shared_ptr<std::mutex> mtx,
                   std::shared_ptr<std::condition_variable> paused_condition,
                   std::shared_ptr<bool> paused)
    : model_(model)
    , data_(data)
    , mtx_(std::move(mtx))
    , paused_condition_(std::move(paused_condition))
    , paused_(std::move(paused))
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
    std::shared_ptr<std::mutex> mtx_;
    std::shared_ptr<std::condition_variable> paused_condition_;
    std::shared_ptr<bool> paused_;
    std::thread* thread_;
    std::atomic<bool> exit_;
    log4cxx::LoggerPtr logger;
    void simulate_loop() {
        LOG4CXX_DEBUG(logger, "Simulate loop starts");
        while(!exit_) {
            std::unique_lock<std::mutex> lock1(*mtx_);
            if(*paused_) {
                LOG4CXX_DEBUG(logger, "Simulate paused");
                paused_condition_->wait(lock1, [this]() -> bool {
                   return !(*paused_);
                });
                LOG4CXX_DEBUG(logger, "Simulate resume");
            }
            mj_step(model_, data_);
            lock1.unlock();
        }
        LOG4CXX_DEBUG(logger, "Simulate loop ends");
    }
};

#endif //MUJOCO_PLAYGROUND_SIMULATE_THREAD_H
