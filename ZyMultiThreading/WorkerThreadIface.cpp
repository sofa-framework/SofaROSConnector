#include "WorkerThreadIface.h"

//#define WORKER_THREAD_IFACE_DEBUG

#ifdef WORKER_THREAD_IFACE_DEBUG
#define DEBOUT(x) x
#else
#define DEBOUT(x)
#endif

using namespace Zyklio::MultiThreading;

WorkerThreadIface::WorkerThreadIface(const std::string& name, priority_type p, size_t stack_size):
m_priority(p),
m_stack_size(stack_size),
m_name(name),
m_state(init),
m_request(rq_none),
sofa::core::objectmodel::BaseObject()
{
}

WorkerThreadIface::~WorkerThreadIface()
{
	BOOST_ASSERT_MSG(detached(), "The thread function must be completed at this point");
}

WorkerThreadIface::event_status WorkerThreadIface::start_event()
{
	DEBOUT(std::cout << "WorkerThread::start_event(" << this->m_name << ")" << std::endl;)

		// no re-run
	if (completed == m_state || paused == m_state)
	{
		DEBOUT(std::cout << " completed or paused" << std::endl;)
			return event_status(false);
	}
	// already runs
	if (running == m_state)
	{
		DEBOUT(std::cout << " already running" << std::endl;)
			return event_status(true, false);
	}
	// try to launch
	// the call may fail due to a system error
	return event_status(launch());
}

bool WorkerThreadIface::start()
{
	event_status rc = start_event();

	if (rc.wait)
		rc.success &= wait_till_launched();

	return rc.success;
}

void WorkerThreadIface::stop(bool force_interrupt)
{
	stop_request(force_interrupt);

	join();
}

void WorkerThreadIface::stop_request(bool force)
{
	DEBOUT(std::cout << "WorkerThread::stop_request(" << this->m_name << ")" << std::endl;)
		// do nothing for completed state
	if (completed == m_state)
	{
		DEBOUT(std::cout << " already completed, do nothing" << std::endl;)
			return;
	}
	// if the thread is not launched
	// just change the state
	if (init == m_state)
	{
		m_state = completed;
		return;
	}

	DEBOUT(std::cout << " emit wake_up event" << std::endl;)
	wake_up(rq_stop);

	// callback is called
	// after the flag is changed
	DEBOUT(std::cout << " on_interrupt()" << std::endl;)
	on_interrupt();

	// interrupt waiting in 
	// any interruption point
	if (force)
		m_thread->interrupt();
}

WorkerThreadIface::event_status WorkerThreadIface::pause_event()
{
	// already paused
	if (paused == m_state)
	{
		return event_status(true, false);
	}
	// cannot pause detached state
	if (detached())
	{
		return event_status(false);
	}

	DEBOUT(std::cout << " request paused state" << std::endl;)
	request(rq_pause);

	// callback is called
	// after the flag is changed
	DEBOUT(std::cout << " on_interrupt()" << std::endl;)
	on_interrupt();

	return event_status(true);
}

bool WorkerThreadIface::pause()
{
	event_status rc = pause_event();

	if (rc.wait)
		rc.success &= wait_till_paused();

	return rc.success;
}

WorkerThreadIface::event_status WorkerThreadIface::resume_event()
{
	DEBOUT(std::cout << "WorkerThread::resume_event(" << this->m_name << ")" << std::endl;)
	// already resumed
	if (running == m_state)
	{
		DEBOUT(std::cout << " already running, do nothing" << std::endl;)
		return event_status(true, false);
	}

	// cannot resume detached state
	if (detached())
	{
		DEBOUT(std::cout << " detached, can't do this" << std::endl;)
		return event_status(false);
	}

	DEBOUT(std::cout << " request resumed state" << std::endl;)
		wake_up(rq_none);

	return event_status(true);
}

bool WorkerThreadIface::resume()
{
	DEBOUT(std::cout << "WorkerThread_Pool::resume(" << this->m_name << ")" << std::endl;)
	WorkerThreadIface::event_status rc = resume_event();

	if (rc.wait)
		rc.success &= wait_till_resumed();

	return rc.success;
}

void WorkerThreadIface::join()
{
	// can throws boost::thread_interrupted 
	// to prevent it we must disable the interruption
	// and in any case we do not want join() complete
	// before the thread
	boost::this_thread::disable_interruption di;
	m_thread->join();
}

WorkerThreadIface::id_type WorkerThreadIface::id() const
{
	return m_id;
}

WorkerThreadIface::handle_type WorkerThreadIface::handle()
{
	// only non constant method version because
	// with the native handle the thread can be
	// alternated "behind the curtain"

	// in detached states (init/completed) handle is not defined
	return detached() ? NULL : (void*)m_thread->native_handle();
}

void WorkerThreadIface::wake_up(erequest rq)
{
	DEBOUT(std::cout << "WorkerThread_Pool<PoolTaskType>::wake_up(" << m_name << "," << rq << ")" << std::endl;)
	// signal that the pause is over
	boost::unique_lock<boost::mutex> lock(m_guard);
	m_request = rq;

	m_pause.notify_one();
}

bool WorkerThreadIface::launch()
{
	boost::thread_attributes attr;
	// set stack size - crossplatform
	attr.set_stack_size(m_stack_size);
	// set thread priority - platform specific
	unsigned rc = set_attribute_priority(m_priority.priority_value(), m_priority.priority_class(), attr.native_handle());
	if (rc)
		m_error.assign(rc, boost::system::system_category());
	// launch void main() member function
	// in separated thread using attriutes

	// can throw boost::thread_resource_error
	try
	{
        msg_info("WorkerThreadIface") << "WorkerThread_Pool<PoolTaskType>::launch(" << m_name << "): Start boost thread";
		m_thread.reset(new boost::thread(attr, boost::bind(&WorkerThreadIface::main, this)));
	}
	catch (boost::thread_resource_error& err)
	{
        msg_error("WorkerThreadIface") << "Thread start exception for " << m_name << ": " << err.what();
		m_error = err.code();
		return false;
	}

	return true;
}

bool WorkerThreadIface::wait_till_launched()
{
    msg_info("WorkerThreadIface") << "wait_till_launched(" << m_name << ") -- state = " << m_state << ", request = " << m_request;
	if (m_request == rq_idle)
	{
		return true;
	}

	// wait till the thread is launched
	{
		boost::unique_lock<boost::mutex> lock(m_guard);
		while (running != m_state && completed != m_state)
		{
			if (m_request == rq_idle)
			{
                msg_info("WorkerThreadIface") << "Requested idle state, assuming as launched (2)...";
				return true;
			}

			m_signal.wait(lock);
            msg_info("WorkerThreadIface") << "wait_till_launched(" << m_name << ")";
		}
	}

	if (running != m_state)
		return false;

	// set priority for already running thread
	// if it is needed
	unsigned rc = adjust_priority(m_priority.priority_value(),
	m_priority.priority_class(), handle());

	if (rc)
		m_error.assign(rc, boost::system::system_category());

	return true;
}

bool WorkerThreadIface::wait_till_resumed()
{
	// wait till the thread is resumed
	{
		boost::unique_lock<boost::mutex> lock(m_guard);

		if (m_request == rq_idle)
		{
            msg_info("WorkerThreadIface") << "wait_till_resumed(" << m_name << "): idle state requested; this counts as resumed (1)";
			return true;
		}

		while (running != m_state && completed != m_state)
		{
            msg_info("WorkerThreadIface") << "wait_till_resumed(" << m_name << ") -- m_request = " << m_request << ", m_state = " << m_state;
			m_signal.wait_for(lock, boost::chrono::milliseconds(100));
			// m_signal.wait_for(lock);

			if (m_request == rq_idle)
			{
                msg_info("WorkerThreadIface") << "wait_till_resumed(" << m_name << "): idle state requested; this counts as resumed (2)";
				return true;
			}
		}
	}

	return running == m_state;
}

bool WorkerThreadIface::wait_till_paused()
{
	// wait till the thread is paused
	{
		boost::unique_lock<boost::mutex> lock(m_guard);
		while (paused != m_state && completed != m_state)
		{
            msg_info("WorkerThreadIface") << "  wait_till_paused(" << m_name << ")";
			m_signal.wait(lock);
		}
	}

	return paused == m_state;
}

void WorkerThreadIface::idle()
{
	// signal paused state
	signal_state(paused);

	// wait in the paused state
	{
		boost::unique_lock<boost::mutex> lock(m_guard);

		while (rq_pause == m_request || rq_idle == m_request)
		{
            msg_info("WorkerThreadIface") << "WorkerThread_Pool<PoolTaskType>::idle(" << m_name << ") -- m_request = " << m_request;
			m_pause.wait(lock);
		}
	}

	// signal running state
	signal_state(running);
}

void WorkerThreadIface::main()
{
    msg_info("WorkerThreadIface") << "WorkerThread_Pool::main(" << this->m_name << ")";
	// read and store current thread id
	m_id = get_current_thread_id();
    msg_info("WorkerThreadIface") << " Got thread id = " << m_id;

	// signal that the thread is running
	signal_state(running);

    msg_info("WorkerThreadIface") << "signalled running/paused state";
	// perform on-start custom action
	on_start();
    msg_info("WorkerThreadIface") << "called on_start()";

	if (m_start_paused)
	{
        msg_info("WorkerThreadIface") << "start paused";
		m_request = rq_idle;
		signal_state(paused);
	}

	// can throw const boost::thread_interrupted
	// if interrupt() was call in any interrupt
	// point
	try
	{
		while (rq_stop != m_request)
		{
			while (rq_none == m_request)
			{
                msg_info("WorkerThreadIface") << " Thread " << m_name << ": no request pending, call action()";
				if (!action())
				{
                    msg_info("WorkerThreadIface") << " Thread " << m_name << ": action() returned false, leave processing loop";
					break;
				}
			}

			if (rq_idle == m_request)
			{
                msg_info("WorkerThreadIface") << " Thread " << m_name << ": idle...";
				idle();
			}
			else if (rq_pause != m_request)
			{
                msg_info("WorkerThreadIface") << " Thread " << m_name << ": received request != rq_pause " << m_request << " vs. " << rq_pause << " ---> leave loop";
				break;
			}

		}
	}
	catch (const boost::thread_interrupted& ex)
	{
		SOFA_UNUSED(ex);
        msg_warning("WorkerThreadIface") << "Thread " << m_name << ": Caught boost::thread_interrupted";
	}

	// update state
	signal_state(completed);

	// perform on-exit custom action
	// after the state was updated
	on_exit();
	// clear id
	m_id = INVALID_THREAD_ID;
}

void WorkerThreadIface::signal_state(state_type state)
{
	// update the state
	// and signal that the thread is 
	// in new state

	boost::unique_lock<boost::mutex> lock(m_guard);

    msg_info("WorkerThreadIface") << "WorkerThread_Pool::signal_state(" << this->m_name << "): new state = " << state << ", old state = " << m_state;

    msg_info("WorkerThreadIface") << "Transition from: ";

	if (m_state == init)
        msg_info("WorkerThreadIface") << " init ";
	else if (m_state == paused)
        msg_info("WorkerThreadIface") << " paused ";
	else if (m_state == running)
        msg_info("WorkerThreadIface") << " running ";
	else if (m_state == completed)
        msg_info("WorkerThreadIface") << " completed ";

    msg_info("WorkerThreadIface") << " to: ";

	if (state == init)
        msg_info("WorkerThreadIface") << " init";
	else if (state == paused)
        msg_info("WorkerThreadIface") << " paused";
	else if (state == running)
        msg_info("WorkerThreadIface") << " running";
	else if (state == completed)
        msg_info("WorkerThreadIface") << " completed";

	if (state == completed)
        msg_info("WorkerThreadIface") << "ALERT state = completed for THREAD " << m_name;

	m_state = state;

	m_signal.notify_one();
}

void WorkerThreadIface::on_interrupt()
{
	boost::unique_lock<boost::mutex> lock(m_guard);
	m_signal.notify_one();
}
