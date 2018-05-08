#ifndef MULTITHREADING_MULTITHREAD_SCHEDULER_H
#define MULTITHREADING_MULTITHREAD_SCHEDULER_H

#include "config_multithreading.h"
#include "Tasks.h"
#include "TaskSchedulerBoostPool.h"

#include <sofa/helper/AdvancedTimer.h>

#define OBBTREEGPU_MULTITHREAD_SCHEDULER_DEBUG

namespace Zyklio
{
    namespace MultiThreading
    {
        template <class PoolTaskType = Zyklio::MultiThreading::PoolTask>
		class TRU_MULTITHREADING_API MultiThread_Scheduler
        {
            public:
				MultiThread_Scheduler(unsigned int numThreads) : m_numThreads(numThreads)
				{
					m_scheduler = new TaskSchedulerPool<PoolTaskType>(m_numThreads);
				}

                ~MultiThread_Scheduler()
				{
					if (m_scheduler != NULL)
					{
						delete m_scheduler;
						m_scheduler = NULL;
					}
				}

                void init()
				{
					std::cout << "ObbTreeGPU_MultiThread_Scheduler::init(): Start " << m_numThreads << " threads." << std::endl;
					m_scheduler->start(m_numThreads);
				}
                
				void bwdInit()
				{
					std::cout << "ObbTreeGPU_MultiThread_Scheduler::bwdInit()" << std::endl;
				}

                void cleanup()
				{
					std::cout << "ObbTreeGPU_MultiThread_Scheduler::cleanup(): Shutdown worker threads" << std::endl;
					m_scheduler->stop();
				}

                bool addTask(PoolTaskType* task)
				{
					m_scheduler->addTask(task);

					return true;
				}

                void suspend()
				{
					m_scheduler->pauseThreads();
				}

                void resume()
				{
					m_scheduler->resumeThreads();
				}

                void runTasks()
				{
#ifdef OBBTREEGPU_MULTITHREAD_SCHEDULER_DEBUG
					std::cout << "=== ObbTreeGPU_MultiThread_Scheduler::runTasks() ===" << std::endl;
#endif

					sofa::helper::AdvancedTimer::stepBegin("MultiThread_Scheduler_Traversal");
					unsigned int waitIterations = 0;
					while (m_scheduler->activeThreads() > 0)
					{
#ifdef OBBTREEGPU_MULTITHREAD_SCHEDULER_DEBUG
						std::cout << "  waiting for running tasks; threads active = " << m_scheduler->activeThreads() << std::endl;
#endif
#ifndef _WIN32
						usleep(50);
#else
						boost::this_thread::sleep_for(boost::chrono::nanoseconds(50));
#endif
						waitIterations++;
					}
					sofa::helper::AdvancedTimer::stepEnd("MultiThread_Scheduler_Traversal");
#ifdef OBBTREEGPU_MULTITHREAD_SCHEDULER_DEBUG
					std::cout << "Task processing finished: Waited " << waitIterations << " loops." << std::endl;
#endif
				}

                void clearTasks()
				{
					m_scheduler->clearProcessedTasks();
					m_scheduler->clearQueuedTasks();
				}

				TaskSchedulerPool<PoolTaskType>* getScheduler()
				{
					return m_scheduler;
				}

				void dumpProcessedTasks()
				{
					m_scheduler->dumpProcessedTasks();
				}

				const unsigned int getNumThreads() const { return m_numThreads; }

            private:
				TaskSchedulerPool<PoolTaskType>* m_scheduler;
                unsigned int m_numThreads;
        };
    }
}

#endif // MULTITHREADING_MULTITHREAD_SCHEDULER_H
