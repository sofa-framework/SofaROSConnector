#ifndef TRU_MULTITHREADING_TASKSCHEDULER_H
#define TRU_MULTITHREADING_TASKSCHEDULER_H

#include "config_multithreading.h"

#include <boost/smart_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>

#include "Tasks.h"
#include "WorkerThreadIface.h"

namespace Zyklio
{
	namespace MultiThreading
    {
		class TRU_MULTITHREADING_API TaskScheduler
        {
            protected:
                enum
                {
                    MAX_THREADS = 16,
                    STACKSIZE = 64*1024 /* 64K */
                };

                volatile unsigned mWorkerCount;

            public:
                virtual bool start(const unsigned int nbThread = 0) = 0;
                virtual bool stop(void) = 0;

                static unsigned GetHardwareThreadsCount();

                unsigned size()	const volatile;
        };
    }
}

#endif //MULTITHREADING_TASKSCHEDULER_H
