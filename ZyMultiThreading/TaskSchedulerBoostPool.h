/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 beta 4      *
*                (c) 2006-2009 MGH, INRIA, USTL, UJF, CNRS                    *
*                                                                             *
* This library is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This library is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this library; if not, write to the Free Software Foundation,     *
* Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA.          *
*******************************************************************************
*                               SOFA :: Modules                               *
*                                                                             *
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#ifndef TRU_MULTITHREADING_TASKSCHEDULERBOOSTPOOL_H
#define TRU_MULTITHREADING_TASKSCHEDULERBOOSTPOOL_H

#include "config_multithreading.h"

#include <boost/smart_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>

#include "Tasks.h"
#include "TaskScheduler.h"

namespace Zyklio
{
	namespace MultiThreading
	{
        template<class PoolTaskType> class WorkerThread_Pool;

        template <class PoolTaskType = PoolTask>
		class TaskSchedulerPool : public TaskScheduler
		{
            public:
                TaskSchedulerPool(int num_threads = 0);
                ~TaskSchedulerPool();

                bool start(const unsigned int NbThread = 0);
                bool stop(void);

				void createWorkerThread(bool paused, const std::string& threadName);
				void dumpProcessedTasks();

				bool startThreads();
				bool stopThreads(bool force_interrupt = false);

				void pauseThreads();
				void resumeThreads();

                /// Return the max number threads that can run concurrently at any
                /// given time using this threadpool.
                int activeThreads();

                void addTask(PoolTaskType* task);

                /// Return a shared pointer to the next task.  If no tasks are
                /// available, return an empty shared pointer.
                bool getNextTask(PoolTaskType *&);
                bool hasTasks();

				void distributeTasks();

                std::vector<PoolTaskType*> &getProcessedTasks();

                void clearProcessedTasks();
                void clearQueuedTasks();

            private:
                TaskSchedulerPool(const TaskSchedulerPool&);
                TaskSchedulerPool& operator=(const TaskSchedulerPool&);

                friend class WorkerThread_Pool<PoolTaskType>;

                boost::mutex m_queue_mutex;
                boost::mutex m_task_mutex;

                std::list<PoolTaskType*> m_queued_tasks;
                std::vector<PoolTaskType*> m_processed_tasks;

                boost::mutex m_mutex;

				void launch_item(WorkerThread_Pool<PoolTaskType>* obj, std::list<WorkerThread_Pool<PoolTaskType>*>& ls);
				void pause_item(WorkerThread_Pool<PoolTaskType>* obj, std::list<WorkerThread_Pool<PoolTaskType>*>& ls);
				void resume_item(WorkerThread_Pool<PoolTaskType>* obj, std::list<WorkerThread_Pool<PoolTaskType>*>& ls);

				std::vector<WorkerThread_Pool<PoolTaskType>*> m_threads;
        };
	} // namespace simulation
} // namespace sofa

#include "TaskSchedulerBoostPool.inl"

#endif // TRU_MULTITHREADING_TASKSCHEDULERBOOSTPOOL_H
