#ifndef MULTITHREADING_WORKERTHREADIFACE_H
#define MULTITHREADING_WORKERTHREADIFACE_H

#include "config_multithreading.h"
#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/helper/system/thread/CTime.h>

#include <boost/system/system_error.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/thread/condition_variable.hpp>

#include <string>

#if defined(linux) || defined(__linux) || defined(__linux__)
#include <sys/syscall.h>
#endif

using namespace sofa::helper::system::thread;

namespace Zyklio
{
	namespace MultiThreading
   {
        class Task;
        class TaskStatus;
        class TaskScheduler;

		//! \class priority_data
		//! \brief Thread priority wrapper
		//! \details The class work with both the platform independent enumerated priority values and
		//! the platform specific values of priority and priority class.
		//! It translates the platform independent enumerated priority values
		//! into platform specific values of priority and priority class using platform defined functions. 
		//! \n Meaning of the priority class also
		//! depends on the platform: for example, for Linux it is scheduling type and for Windows - thread
		//! priority class.
		class priority_data
		{
			public:

				//! platform independent enumerated priority values
				enum epriority
				{
					//! Normal thread priority
					NORMAL = 0,
					//! Thread prioity above the normal
					ABOVE,
					//! Thread prioity below the normal
					BELOW,
					//! High thread prioity
					HIGH,
					//! Low thread prioity
					LOW,
					//! Real time thread prioity
					RT,
					//! Idle process thread prioity
					IDLE
				};

				// differnt from any priority value
				enum { PRIORITY_DEFAULT_VALUE = 999999 };

				//! default ctor
				//! uses the default values for both priority and class
				priority_data() :
					m_value(PRIORITY_DEFAULT_VALUE),
					m_class(PRIORITY_DEFAULT_VALUE)
				{}

				//! ctor gets numerical platform specific
				//! priority value and uses the default
				//! priority class
				//! \param pr numerical platform specific
				//! priority value
				priority_data(int pr) :
					m_value(pr), m_class(PRIORITY_DEFAULT_VALUE)
				{}

				//! ctor gets numerical platform specific
				//! values for priority and priority class
				//! \param pr numerical platform specific
				//! priority value
				//! \param cl numerical platform specific
				//! priority class value
				priority_data(int pr, int cl) :
					m_value(pr), m_class(cl)
				{}

				//! ctor gets enumerated platform independent
				//! priority value and converts it to platform
				//! specific values
				priority_data(epriority ep);

				//! gets the platform specific priority value
				int priority_value() const { return m_value; }
				//! gets the platform specific priority class value
				int priority_class() const { return m_class; }
			private:
				const int m_value;
				const int m_class;
		};

		static const size_t DEFAULT_STACK_VALUE = 0;

		typedef unsigned int thread_id_t;
		const thread_id_t INVALID_THREAD_ID = 0;

		typedef void* handle_t;
		const handle_t INVALID_THREAD_HANDLE = 0;

		static thread_id_t get_current_thread_id()
		{
#ifndef _WIN32
#if defined(linux) || defined(__linux) || defined(__linux__)
			// is not cross platform 
			// works only for Linux
			return syscall(SYS_gettid);
#else
			// cross platform
			// actually thread handle
			return pthread_self();
#endif
#else
			thread_id_t thread_id = GetCurrentThreadId();
			return thread_id;
#endif
		}

		static unsigned adjust_priority(int priority_value, int priority_class, Zyklio::MultiThreading::handle_t h)
		{
#ifndef _WIN32
			SOFA_UNUSED(priority_value);
			SOFA_UNUSED(priority_class);
			SOFA_UNUSED(h);
			return 0;
#else
			if (priority_data::PRIORITY_DEFAULT_VALUE != priority_value)
			{
				if (!SetThreadPriority(h, priority_value))
					return GetLastError();
			}

			if (priority_data::PRIORITY_DEFAULT_VALUE != priority_class)
			{
				if (!SetPriorityClass(h, priority_class))
					return GetLastError();
			}

			return 0;
#endif
		}

		static unsigned set_attribute_priority(int priority_value, int priority_class, Zyklio::MultiThreading::handle_t h)
		{
#ifndef _WIN32
			// for the default priority we should use SCHED_OTHER class
			// for others, if the class is not specified - SCHED_RR class
			// Otherwise use the specified values
			if (priority_data::PRIORITY_DEFAULT_VALUE == priority_class)
			{
				if (priority_data::PRIORITY_DEFAULT_VALUE == priority_value)
				{
					priority_class = SCHED_OTHER;
					priority_value = 0;
				}
				else
				{
					priority_class = SCHED_RR;
				}
			}

			unsigned rc = pthread_attr_setschedpolicy(static_cast<pthread_attr_t*>(h), priority_class);

			if (rc) return rc;

			struct sched_param params;
			memset(&params, 0, sizeof(params));
			params.sched_priority = priority_value;

			return pthread_attr_setschedparam(static_cast<pthread_attr_t*>(h), &params);
#else
			return 0;
#endif
		}

		class TRU_MULTITHREADING_API WorkerThreadIface: public sofa::core::objectmodel::BaseObject
        {
			public:
				typedef unsigned int id_type;
				typedef void* handle_type;

				typedef priority_data priority_type;
				typedef boost::system::error_code error_type;

				//! thread states
				enum state_type
				{
					//! thread did not start yet
					init,
					//! thread is paused
					paused,
					//! thread is running
					running,
					//! thread function is completed
					completed
				};

				WorkerThreadIface(const std::string& name = std::string(),
								  priority_type p = priority_type(),
								  size_t stack_size = DEFAULT_STACK_VALUE);

				//! starts the thread. The method is blocking.
				bool start();

				//! stops the thread. The method is blocking.
				//! \param force_interrupt forces using the boost's interruption points:
				//! if the thread in waitng at any of the boost's interruption points, it
				//! will be interrupted as well.
				void stop(bool force_interrupt = false);

				//! pauses the thread. The method is blocking.
				bool pause();

				//! resumes the thread. The method is blocking.
				bool resume();

				//! waits till the thread is completed. The method is blocking.
				void join();

				//! gets the platform specific thread id
				id_type id() const;

				//! gets the platform specific thread handle
				handle_type handle();

				//! gets the thread current state
				state_type state() const
				{
					return m_state;
				}

				bool detached() const
				{
					return init == m_state || completed == m_state;
				}

				~WorkerThreadIface();

				const std::string& name() const
				{
					return m_name;
				}

			protected:

				//! the action callback implemented in the derived class.
				//! The callback will be called in the context of the
				//! running thread again and again till it returns true.
				//! To terminate the thread the callback should return false.
				//! The callback <b>must</b> exit if the <code>is_interrupted()</code> 
				//! flag is set to <code>true</code>.
				//! The method is pure virtual and must be implemented in derived classes.
				//! \return code>true</code> to continue the execution and
				//! code>false</code> to complete the thread function.
				virtual bool action() { return true; }

				//! the callback is called after the thread started
				//! its execution. The thread state is <code>running</code>.
				//! The method is called in context of the running thread.
				//! The method can be implemented in derived classes.
				virtual void on_start(){}

				//! the callback is called after the thread ceased to call
				//! the <code>action</code> method but
				//! before the thread function is completed.
				//! The method is called in context of the running thread.
				//! The thread state is <code>completed</code>.
				//! The method can be implemented in derived classes.
				virtual void on_exit(){}

				//! the callback is invoked each time either the <code>stop()</code>
				//! or <code>pause()</code> is called. A derived class can use this
				//! callback to force the <code>action()</code> to check 
				//! the<code>is_interrupted()</code> flag.
				virtual void on_interrupt();

				//! signals to the <code>action()</code> to exit.
				//! The flag is set on either stop or pause request.
				bool is_interrupted() const
				{
					return rq_none != m_request;
				}

			public:
				inline void startThreadTimer()
				{
					m_threadStart = CTime::getTime();
				}

				inline void stopThreadTimer()
				{
					m_threadStop = CTime::getTime();
					m_threadRunTime = m_threadStop - m_threadStart;
				}

				inline void resetThreadTimer()
				{
					m_threadStart = m_threadStop = CTime::getTime();
				}

				inline ctime_t getThreadRunTime()
				{
					m_threadRunTime = m_threadStop - m_threadStart;
					return m_threadRunTime;
				}

				inline void startStepTimer()
				{
					m_stepStart = CTime::getTime();
				}
				
				inline void stopStepTimer()
				{
					m_stepStop = CTime::getTime();
					m_stepRunTime = m_stepStop - m_stepStart;
				}

				inline void resetStepTimer()
				{
					m_stepStart = m_stepStop = CTime::getTime();
				}

				inline ctime_t getStepRunTime()
				{
					return m_stepRunTime;
				}

			protected:
				ctime_t m_threadStart;
				ctime_t m_threadStop;

				ctime_t m_threadRunTime;

				ctime_t m_stepStart;
				ctime_t m_stepStop;

				ctime_t m_stepRunTime;

				boost::shared_ptr<boost::thread> m_thread;

				virtual void main();

				void idle();
				void signal_state(state_type state);
				enum erequest{ rq_none, rq_pause, rq_stop, rq_idle };

				void request(erequest rq)
				{
					m_request = rq;
				}

				void stop_request(bool force);
				bool launch();
				void wake_up(erequest rq);

				bool wait_till_launched();
				bool wait_till_resumed();
				bool wait_till_paused();

				struct event_status
				{
					event_status() :
					success(false),
					wait(false)
					{}

					event_status(bool success_, bool wait_) :
						success(success_),
						wait(wait_)
					{}

					event_status(bool rc) :
						success(rc),
						wait(rc)
					{}

					bool success;
					bool wait;
				};

				event_status start_event();
				event_status pause_event();
				event_status resume_event();

				const std::string m_name;
				const priority_type m_priority;
				const size_t m_stack_size;
				volatile state_type m_state;
				volatile erequest m_request;
				volatile id_type m_id;
				boost::system::error_code m_error;
				boost::mutex m_guard;
				boost::condition_variable m_signal;
				boost::condition_variable m_pause;

				bool m_start_paused;
		};
    }
}

#endif //MULTITHREADING_WORKERTHREADIFACE_H
