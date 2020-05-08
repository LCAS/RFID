#ifndef __CONSUMERPRODUCERQUEUE_H__
#define __CONSUMERPRODUCERQUEUE_H__

#include <queue>
#include <mutex>
#include <condition_variable>

/*
 * BASED ON THE CODE FROM dpressel at https://gist.github.com/dpressel/de9ea7603fa3f20b55bf
 * 
 * Some references in order
 *
 * Some code I wrote a long time before C++ 11 to do consumer producer buffers, using 2 condition variables
 * https://github.com/mdaus/coda-oss/blob/master/modules/c%2B%2B/mt/include/mt/RequestQueue.h
 *
 * A great article explaining both 2 condition variable and 1 condition variable buffers
 * https://en.wikipedia.org/wiki/Monitor_%28synchronization%29#Condition_variables
 *
 * C++ 11 thread reference:
 * http://en.cppreference.com/w/cpp/thread
 */

template<typename T>
class ConsumerProducerQueue{

        std::condition_variable store_cond_;
        std::mutex store_mutex_;
        std::queue<T> data_queue_;
        int maxSize_;
    public:
        ConsumerProducerQueue() : maxSize_(1000)
        { }

        ConsumerProducerQueue(int mxsz) : maxSize_(mxsz)
        { }

        void add(T request){
            std::unique_lock<std::mutex> lock(store_mutex_);
            store_cond_.wait(lock, [this]()
            { return !isFull(); });
            data_queue_.push(request);
            lock.unlock();
            store_cond_.notify_all();
        }

        void consume(T &request){
            std::unique_lock<std::mutex> lock(store_mutex_);
            store_cond_.wait(lock, [this]()
            { return !isEmpty(); });
            request = data_queue_.front();
            data_queue_.pop();
            lock.unlock();
            store_cond_.notify_all();
        }

        bool isEmpty() const{
            return data_queue_.size() == 0;
        }

        void waitEmpty(){
            std::unique_lock<std::mutex> lock(store_mutex_);
            store_cond_.wait(lock, [this]()
            { return isEmpty(); });
        }

        int length() const{
            return data_queue_.size();
        }

        void clear(){
            std::unique_lock<std::mutex> lock(store_mutex_);
            while (!isEmpty()){
                data_queue_.pop();
            }
            lock.unlock();
            store_cond_.notify_all();
        }

        bool isFull() const{
        return data_queue_.size() >= maxSize_;
    }
};

#endif