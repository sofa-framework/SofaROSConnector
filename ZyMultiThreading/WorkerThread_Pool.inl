#include "WorkerThread_Pool.h"

#include "TaskSchedulerBoostPool.h"

//#define WORKER_THREAD_POOL_DEBUG

#ifdef WORKER_THREAD_POOL_DEBUG
#define DEBOUT(x) x
#else
#define DEBOUT(x)
#endif

#ifndef _WIN32
#if defined(linux) || defined(__linux) || defined(__linux__)
#include <sys/syscall.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#endif
#endif

using namespace Zyklio::MultiThreading;

template <class PoolTaskType>
WorkerThread_Pool<PoolTaskType>::WorkerThread_Pool(TaskSchedulerPool<PoolTaskType> *scheduler, const std::string& name, priority_type p, size_t stack_size, bool paused) :
	WorkerThreadIface(name, p, stack_size),
	m_scheduler(scheduler),
	m_nextTaskAvailable(false),
	m_start_paused(paused)
{
}

template <class PoolTaskType>
WorkerThread_Pool<PoolTaskType>::~WorkerThread_Pool()
{
	BOOST_ASSERT_MSG(detached(), "The thread function must be completed at this point");
}

template <class PoolTaskType>
void WorkerThread_Pool<PoolTaskType>::enqueueTask(PoolTaskType* task)
{
	boost::unique_lock<boost::mutex> lock(m_guard);
	if (task != NULL)
	{
		//std::cout << "WorkerThread_Pool::enqueueTask(" << task->getTaskID() << ")" << std::endl;
		this->m_taskList.push_back(task);
	}
}

template <class PoolTaskType>
bool WorkerThread_Pool<PoolTaskType>::action()
{
	boost::unique_lock<boost::mutex> lock(m_guard);

	if (this->m_taskList.size() > 0)
	{
        DEBOUT(std::cout << "=== WorkerThread_Pool::action(" << this->name() << ") -- Task list preset: " << m_taskList.size() << " tasks in list ===" << std::endl;)
		unsigned int numProcessedTasks = 0;
		for (size_t k = 0; k < m_taskList.size(); k++)
		{
			if (!m_taskList[k]->isFinished())
			{
                DEBOUT(std::cout << "WorkerThread_Pool::action(" << this->name() << "): Process task '" << m_taskList[k]->getTaskID() << "'" << std::endl;)

				m_taskList[k]->run(this);
				m_taskList[k]->signalFinished();
				
                DEBOUT(std::cout << "  task run complete" << std::endl;)
				
				m_processedTaskIDs.push_back(m_taskList[k]->getTaskID());
				numProcessedTasks++;

				if (is_interrupted())
					return true;
			}
		}

		if (numProcessedTasks == 0)
		{
            DEBOUT(std::cout << " No tasks processed in last iteration; request idle state" << std::endl;)
			this->request(rq_idle);
            DEBOUT(std::cout << " exit action()" << std::endl;)
			return false;
			//return true;
		}
		//return true;
	}
	else
	{
        DEBOUT(std::cout << "=== WorkerThread_Pool::action(" << this->name() << ") -- NO TASK LIST PRESET: Try to get tasks from scheduler===" << std::endl;)
		do
		{
			m_nextTaskAvailable = m_scheduler->hasTasks();

            DEBOUT(std::cout << "WorkerThread_Pool::action(" << this->m_name << "): Check if scheduler has tasks available: " << m_nextTaskAvailable << std::endl;)

			if (m_nextTaskAvailable)
			{
				std::cout << "WorkerThread_Pool::action(" << this->m_name << "): Get next task from scheduler." << std::endl;
				boost::unique_lock<boost::mutex> lock(m_scheduler->m_queue_mutex);
				m_nextTaskAvailable = m_scheduler->getNextTask(m_task);

				if (m_task != NULL)
				{
                    DEBOUT(std::cout << "WorkerThread_Pool::action(" << this->name() << "): Process task '" << m_task->getTaskID() << "'" << std::endl;)

					if (!m_task->isFinished())
					{
						m_task->run(this);
						m_task->signalFinished();
                        DEBOUT(std::cout << "  task run complete" << std::endl;)
						m_processedTaskIDs.push_back(m_task->getTaskID());
					}
				}
				else
				{
                    DEBOUT(std::cout << " NULL task pointer received from scheduler, should not happen!" << std::endl;)
					this->request(rq_idle);
					return false;
					//this->signal_state(paused);
				}
			}
			else
			{
                DEBOUT(std::cout << "WorkerThread_Pool::action(" << this->name() << "): No more tasks available from scheduler!" << std::endl;) //, waiting for pause/resume request" << std::endl;
				// m_signal.wait(lock);
				this->request(rq_idle);
				return false;
				//this->signal_state(paused);
			}
		} while (!is_interrupted() && m_nextTaskAvailable);
	}

	return true;
}