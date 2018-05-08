#include "TaskSchedulerBoostPool.h"
#include "WorkerThread_Pool.h"

#include <sofa/helper/system/thread/CTime.h>

#include <boost/chrono.hpp>

#ifdef TASK_SCHEDULAR_BOOST_POOL_DEBUG
#define DEBOUT(x) x
#else
#define DEBOUT(x)
#endif

namespace Zyklio
{
	namespace MultiThreading
    {
        template <class PoolTaskType>
        TaskSchedulerPool<PoolTaskType>::TaskSchedulerPool(int num_threads)
        {

		}

        template <class PoolTaskType>
        TaskSchedulerPool<PoolTaskType>::~TaskSchedulerPool()
        {
            DEBOUT(std::cout << "TaskSchedulerPool::~TaskSchedulerPool(): Waiting for workers to join" << std::endl;)
           
			this->stopThreads(true);
            for (typename std::vector<WorkerThread_Pool<PoolTaskType>*>::iterator begin = m_threads.begin(); begin != m_threads.end(); ++begin)
			{
				if ((*begin) != NULL)
				{
					delete (*begin);
					*begin = NULL;
				}
			}
			m_threads.clear();
        }

		// TODO FA: Use a wait condition to signal all threads finishing!
        template <class PoolTaskType>
        int TaskSchedulerPool<PoolTaskType>::activeThreads()
        {
            DEBOUT(std::cout << "TaskSchedulerPool<PoolTaskType>::activeThreads()" << std::endl;)
			int activeThreads = 0;
			for (size_t k = 0; k < m_threads.size(); k++)
			{
				WorkerThread_Pool<PoolTaskType>* thread = m_threads.at(k);
                DEBOUT(std::cout << " thread " << k << " (" << thread->name() << ") << state query" << std::endl;)
				if (thread->state() == WorkerThread_Pool<PoolTaskType>::running)
				{
                    DEBOUT(std::cout << " running" << std::endl;)
					activeThreads++;
				}
				else
				{
                    DEBOUT(std::cout << " not running" << std::endl;)
				}
			}


            DEBOUT(std::cout << " activeThreads = " << activeThreads << std::endl;)
			return activeThreads;
        }

        // Add a task that is being tracked by a shared pointer.
		/// TODO: GLEICHMAESSIG VERTEILEN auf Worker-Threads, am besten bevor resumeThreads() aufgerufen wird!!!
        template <class PoolTaskType>
        void TaskSchedulerPool<PoolTaskType>::addTask(PoolTaskType* task)
        {
            //std::cout << "TaskSchedulerPool::addTask('" << task->getTaskID() << "'); m_paused = " << m_paused << std::endl;
            {
                m_mutex.lock();
                m_queued_tasks.push_back(task);
                m_mutex.unlock();
            }
            /*if (!m_paused)
            {
                std::cout << "  not in pause state, notifying workers" << std::endl;
                this->notify();
            }*/
        }

        template <class PoolTaskType>
        bool TaskSchedulerPool<PoolTaskType>::hasTasks()
        {
            boost::unique_lock<boost::mutex> lock(m_mutex);
            return (!m_queued_tasks.empty());
        }

        template <class PoolTaskType>
        bool TaskSchedulerPool<PoolTaskType>::getNextTask(PoolTaskType*& nextTask)
        {
            DEBOUT(std::cout << "TaskSchedulerPool::getNextTask()" << std::endl;)
            if (m_queued_tasks.empty())
            {
                DEBOUT(std::cout << " task queue empty, return false" << std::endl;)
                nextTask = NULL;
                return false;
            }

            if (m_task_mutex.try_lock())
            {
                DEBOUT(std::cout << " mutex lock acquired" << std::endl;)
                PoolTaskType* task = m_queued_tasks.front();

                DEBOUT(std::cout << " next task: " << task->getTaskID() << std::endl;)
                m_processed_tasks.push_back(task);
                DEBOUT(std::cout << " task on processed_tasks list: " << m_processed_tasks.at(m_processed_tasks.size() - 1)->getTaskID() << std::endl;)
                nextTask = task;
                m_queued_tasks.pop_front();

                m_task_mutex.unlock();

                return true;
            }
            else
            {
                DEBOUT(std::cout << " FAILED to lock mutex for task queue access!" << std::endl;)
                nextTask = NULL;
                return false;
            }
        }

        template <class PoolTaskType>
        bool TaskSchedulerPool<PoolTaskType>::start(const unsigned int NbThread)
        {
            DEBOUT(std::cout << "TaskSchedulerPool::start(" << NbThread << ") -- No-op" << std::endl;)
            return true;
        }

        template <class PoolTaskType>
		bool TaskSchedulerPool<PoolTaskType>::stop()
		{
			DEBOUT(std::cout << "TaskSchedulerPool::stop() -- No-op" << std::endl;)
				return true;
		}

        template <class PoolTaskType>
        std::vector<PoolTaskType*>& TaskSchedulerPool<PoolTaskType>::getProcessedTasks()
        {
            boost::unique_lock<boost::mutex> lock(m_mutex);
            return m_processed_tasks;
        }

        template <class PoolTaskType>
        void TaskSchedulerPool<PoolTaskType>::clearProcessedTasks()
        {
            boost::unique_lock<boost::mutex> lock(m_mutex);
            if (m_processed_tasks.size() > 0)
            {
                DEBOUT(std::cout << "TaskSchedulerPool::clearProcessedTasks(): delete " << m_processed_tasks.size() << " tasks." << std::endl;)
                m_processed_tasks.clear();
            }

			for (unsigned int k = 0; k < m_threads.size(); k++)
			{
				m_threads[k]->clearTaskList();
				m_threads[k]->clearProcessedTaskIDs();
			}
        }

        template <class PoolTaskType>
        void TaskSchedulerPool<PoolTaskType>::clearQueuedTasks()
        {
            boost::unique_lock<boost::mutex> lock(m_mutex);
            if (m_queued_tasks.size() > 0)
            {
                DEBOUT(std::cout << "TaskSchedulerPool::clearQueuedTasks(): delete " << m_queued_tasks.size() << " tasks." << std::endl;)
                m_queued_tasks.clear();
            }
        }

		template <class PoolTaskType>
		void TaskSchedulerPool<PoolTaskType>::launch_item(WorkerThread_Pool<PoolTaskType>* obj, std::list<WorkerThread_Pool<PoolTaskType>*>& ls)
		{
            DEBOUT(std::cout << "TaskSchedulerPool<PoolTaskType>::launch_item(" << obj->name() << ")" << std::endl;)
            typename WorkerThread_Pool<PoolTaskType>::event_status rc = obj->start_event();
			
			if (rc.wait) 
				ls.push_back(obj);
		}

		template <class PoolTaskType>
		void TaskSchedulerPool<PoolTaskType>::pause_item(WorkerThread_Pool<PoolTaskType>* obj, std::list<WorkerThread_Pool<PoolTaskType>*>& ls)
		{
            DEBOUT(std::cout << "TaskSchedulerPool<PoolTaskType>::pause_item(" << obj->name() << ")" << std::endl;)
            typename WorkerThread_Pool<PoolTaskType>::event_status rc = obj->pause_event();
			
			if (rc.wait) 
				ls.push_back(obj);
		}

		template <class PoolTaskType>
		void TaskSchedulerPool<PoolTaskType>::resume_item(WorkerThread_Pool<PoolTaskType>* obj, std::list<WorkerThread_Pool<PoolTaskType>*>& ls)
		{
            typename WorkerThread_Pool<PoolTaskType>::event_status rc = obj->resume_event();
			
			if (rc.wait) 
				ls.push_back(obj);
		}

		template <class PoolTaskType>
		bool TaskSchedulerPool<PoolTaskType>::startThreads()
		{
            DEBOUT(std::cout << "TaskSchedulerPool<PoolTaskType>::startThreads()" << std::endl;)
			std::list<WorkerThread_Pool<PoolTaskType>*> waiting_list;

            typename std::vector<WorkerThread_Pool<PoolTaskType>*>::iterator begin = m_threads.begin();
            for (; begin != m_threads.end(); ++begin)
			{
                DEBOUT(std::cout << " - start thread " << (*begin)->name() << std::endl;)
				launch_item((*begin), waiting_list);
			}

			std::for_each(waiting_list.begin(), waiting_list.end(),
				boost::mem_fn(&WorkerThread_Pool<PoolTaskType>::wait_till_launched));


			return true;
		}

		template <class PoolTaskType>
		bool TaskSchedulerPool<PoolTaskType>::stopThreads(bool force_interrupt)
		{
            DEBOUT(std::cout << "TaskSchedulerPool<PoolTaskType>::stopThreads(" << force_interrupt << "): Worker thread count = " << m_threads.size() << std::endl;)

            typename std::vector<WorkerThread_Pool<PoolTaskType>*>::iterator item = m_threads.begin();
            for (; item != m_threads.end(); ++item)
			{
                DEBOUT(std::cout << " - stop thread " << (*item)->name() << std::endl;)
				(*item)->stop_request(force_interrupt);
			}

            for (typename std::vector<WorkerThread_Pool<PoolTaskType>*>::iterator item = m_threads.begin(); item != m_threads.end(); ++item)
				(*item)->join();

			return true;
		}

		template <class PoolTaskType>
		void TaskSchedulerPool<PoolTaskType>::pauseThreads()
		{
            DEBOUT(std::cout << "TaskSchedulerPool<PoolTaskType>::pauseThreads()" << std::endl;)

			dumpProcessedTasks();

			std::list<WorkerThread_Pool<PoolTaskType>*> waiting_list;
            for (typename std::vector<WorkerThread_Pool<PoolTaskType>*>::iterator begin = m_threads.begin(); begin != m_threads.end(); ++begin)
			{
                DEBOUT(std::cout << " - pause thread " << (*begin)->name() << std::endl;)
				pause_item((*begin), waiting_list);
			}

			std::for_each(waiting_list.begin(), waiting_list.end(),
				boost::mem_fn(&WorkerThread_Pool<PoolTaskType>::wait_till_paused));
		}

		template <class PoolTaskType>
		void TaskSchedulerPool<PoolTaskType>::resumeThreads()
		{
            DEBOUT(std::cout << "TaskSchedulerPool<PoolTaskType>::resumeThreads()" << std::endl;)

			std::list<WorkerThread_Pool<PoolTaskType>*> waiting_list;
            for (typename std::vector<WorkerThread_Pool<PoolTaskType>*>::iterator begin = m_threads.begin(); begin != m_threads.end(); ++begin)
			{
                DEBOUT(std::cout << " - resume thread " << (*begin)->name() << std::endl;)
				resume_item((*begin), waiting_list);
			}

			std::for_each(waiting_list.begin(), waiting_list.end(),
				boost::mem_fn(&WorkerThread_Pool<PoolTaskType>::wait_till_resumed));
		}

		template <class PoolTaskType>
		void TaskSchedulerPool<PoolTaskType>::createWorkerThread(bool paused, const std::string& threadName)
		{
			std::stringstream thread_name_stream;
			thread_name_stream << threadName << "_WorkerThread_" << m_threads.size();
			WorkerThread_Pool<PoolTaskType>* next_worker = new WorkerThread_Pool<PoolTaskType>(this, thread_name_stream.str(), 
                typename WorkerThread_Pool<PoolTaskType>::priority_type(),
#ifdef _WIN32
				Zyklio::MultiThreading::DEFAULT_STACK_VALUE
#else
                0
#endif
                , paused);

			m_threads.push_back(next_worker);
		}

		template <class PoolTaskType>
		void TaskSchedulerPool<PoolTaskType>::dumpProcessedTasks()
		{
            DEBOUT(
			for (std::vector<WorkerThread_Pool<PoolTaskType>*>::iterator begin = m_threads.begin(); begin != m_threads.end(); ++begin)
			{
                std::cout << " - thread " << (*begin)->name() << ": ";
				for (unsigned int k = 0; k < (*begin)->m_processedTaskIDs.size(); k++)
					std::cout << (*begin)->m_processedTaskIDs[k] << ";";

				std::cout << std::endl;
            })
		}

		template <class PoolTaskType>
		void TaskSchedulerPool<PoolTaskType>::distributeTasks()
		{
            typename std::vector<WorkerThread_Pool<PoolTaskType>*>::iterator thread_it = this->m_threads.begin();
            for (typename std::list<PoolTaskType*>::iterator begin = m_queued_tasks.begin(); begin != m_queued_tasks.end(); ++begin)
			{
				(*thread_it)->enqueueTask(*begin);

				thread_it++;
				if (thread_it == m_threads.end())
					thread_it = m_threads.begin();
			}
		}
    } // namespace MultiThreading
} // namespace Zyklio

