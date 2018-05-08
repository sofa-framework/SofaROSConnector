#ifndef TRU_MULTITHREADING_WORKERTHREAD_POOL_H
#define TRU_MULTITHREADING_WORKERTHREAD_POOL_H

#include "WorkerThreadIface.h"
#include "Tasks.h"

#include "TaskSchedulerBoostPool.h"

#include <boost/chrono.hpp>
#include <boost/system/system_error.hpp>

#include <sofa/core/objectmodel/BaseObject.h>


#include <set>
#include <streambuf>

namespace Zyklio
{
	namespace MultiThreading
    {
        class TaskScheduler;

		/**
		Thread class for task-based processing 
		Runs as long as there are tasks available to process.
		If given task list has been processed, enters idle state.
		*/
        template <class PoolTaskType = PoolTask>
		class TRU_MULTITHREADING_API WorkerThread_Pool : public WorkerThreadIface
        {
			public:
				WorkerThread_Pool(TaskSchedulerPool<PoolTaskType> *scheduler,
								  const std::string& name = std::string(),
								  priority_type p = priority_type(),
								  size_t stack_size = DEFAULT_STACK_VALUE,
								  bool startIdle = false);

                ~WorkerThread_Pool();


				void clearProcessedTaskIDs()
				{
					m_processedTaskIDs.clear();
				}

				void clearTaskList()
				{
					this->m_taskList.clear();
				}

				void enqueueTask(PoolTaskType*);

			protected:
				bool action();

            private:
				friend class TaskSchedulerPool<PoolTaskType>;

                PoolTaskType* m_task;
				std::vector<PoolTaskType*> m_taskList;

                TaskSchedulerPool<PoolTaskType>* m_scheduler;
                
                bool m_nextTaskAvailable;

				boost::chrono::thread_clock::time_point m_threadStart;
				boost::chrono::thread_clock::time_point m_threadStop;
		
				boost::mutex m_guard;
				std::vector<std::string> m_processedTaskIDs;

				bool m_start_paused;
        };
    }
}

#include "WorkerThread_Pool.inl"

#endif // TRU_MULTITHREADING_WORKERTHREAD_POOL_H
