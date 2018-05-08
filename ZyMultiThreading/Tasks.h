/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, development version     *
*                (c) 2006-2016 INRIA, USTL, UJF, CNRS, MGH                    *
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
#ifndef MultiThreadingTasks_h__
#define MultiThreadingTasks_h__

#include <boost/detail/atomic_count.hpp>
#include <boost/pool/singleton_pool.hpp>

#include <sofa/helper/system/atomic.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>

#include "config_multithreading.h"
#include "WorkerThreadIface.h"

namespace Zyklio
{
	namespace MultiThreading
	{
        // Task Status class definition
        class TaskStatus
        {
            public:
                TaskStatus(): mBusy(0L)
                {

                }

                ~TaskStatus()
                {
                    //std::cout << "TaskStatus::~TaskStatus(): mBusy = " << mBusy.operator long() << std::endl;
                }

                bool IsBusy() const;

                void MarkBusy(bool bBusy);

            private:
                boost::detail::atomic_count mBusy;
        };

		class TRU_MULTITHREADING_API PoolTask
        {
            public:
				PoolTask() : m_finished(false), m_taskId("Unnamed PoolTask")
                {
                    //std::cout << "PoolTask::PoolTask()" << std::endl;
                }

                PoolTask(const PoolTask &other)
                {
                    if (this != &other)
                    {
                        this->m_finished = false;
                        this->m_taskId = other.m_taskId;
                    }
                }

                PoolTask& operator=(const PoolTask& other)
                {
                    if (this != &other)
                    {
                        this->m_finished = false;
                        this->m_taskId = other.m_taskId;
                    }
                    return *this;
                }

                virtual bool run(WorkerThreadIface*)
                {
                    //std::cout << "PoolTask::run(" << this->m_taskId << ")" << std::endl;
					return true;
                }

                void signalFinished()
                {
                    boost::unique_lock<boost::mutex> lock(m_task_mutex);
                    m_finished = true;
                    m_finished_event.notify_all();
                }

				bool isFinished()
				{
					boost::unique_lock<boost::mutex> lock(m_task_mutex);
					return m_finished;
				}

				void setFinished(bool finished)
				{
					boost::unique_lock<boost::mutex> lock(m_task_mutex);
					m_finished = finished;
				}

                void join()
                {
                    while (true)
                    {
                      boost::unique_lock<boost::mutex> lock(m_task_mutex);

                      if (m_finished)
                          return;

                      m_finished_event.wait(lock);
                    }
                }

                const std::string& getTaskID() { return m_taskId; }
                void setTaskID(const std::string& id) { m_taskId = id; }

            protected:
                boost::mutex m_task_mutex;
                boost::condition m_finished_event;
                volatile bool m_finished;
                std::string m_taskId;
        };
	} // namespace simulation
} // namespace sofa


#include "Tasks.inl"

#endif // MultiThreadingTasks_h__
