#ifndef THREAD_HPP
#define THREAD_HPP

#include <iostream>
#include <pthread.h>
#include <sched.h>

class thread
{
protected:
    pthread_t tid;
    bool _useFIFO;
    pthread_attr_t attr;
private:
    static void *start_thread(void *arg)
    {
        thread *ptr = (thread *)arg;
        ptr->run();
        return (void *) ptr;
    }
public:
    int start()
    {
        if(_useFIFO)
        {
            pthread_attr_init(&attr);
            pthread_attr_setschedpolicy(&attr,SCHED_FIFO);
            struct sched_param param;
            param.sched_priority = 1;
            pthread_attr_setschedparam(&attr, &param);
            if (pthread_create(&tid, &attr, start_thread, (void *)this) != 0)
            {
                std::cout << "pthread create error" << std::endl;
                return -1;
            }
        }
        
        if (pthread_create(&tid, NULL, start_thread, (void *)this) != 0)
        {
            std::cout << "pthread create error" << std::endl;
            return -1;
        }
        return tid;
    }
    virtual void run() = 0;
    
    thread(const bool& useFIFO = false):_useFIFO(useFIFO) {}
};

#endif