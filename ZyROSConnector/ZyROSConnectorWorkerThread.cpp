#include "ZyROSConnectorWorkerThread.h"

using namespace Zyklio::MultiThreading;
using namespace Zyklio::ROSConnector;

ZyROSConnectorWorkerThread::ZyROSConnectorWorkerThread(ZyROSConnector* pCon) : WorkerThread_SingleTask(), m_rosConnector(pCon)
{
	// Run directly after start call, no initial pause necessary
	m_start_paused = false;
	m_func = &ZyROSConnector::rosLoop;
}

ZyROSConnectorWorkerThread::~ZyROSConnectorWorkerThread()
{

}

bool ZyROSConnectorWorkerThread::addTopicListener(const boost::shared_ptr<ZyROSListener>& subscriber)
{
	boost::mutex::scoped_lock lock(m_mutex);
	for (std::vector<boost::shared_ptr<ZyROSListener> >::const_iterator it = m_topicSubscribers.begin(); it != m_topicSubscribers.end(); ++it)
	{
		if (subscriber->getUuid() == (*it)->getUuid())
		{
			return false;
		}
	}

	m_topicSubscribers.push_back(boost::move(subscriber));
	m_activeSubscribers.push_back(true);
	return true;
}

bool ZyROSConnectorWorkerThread::removeTopicListener(boost::shared_ptr<ZyROSListener>& subscriber)
{
	boost::mutex::scoped_lock lock(m_mutex);

	size_t subscriber_pos = -1, subscriber_idx = 0;
	for (std::vector<boost::shared_ptr<ZyROSListener> >::iterator it = m_topicSubscribers.begin(); it != m_topicSubscribers.end(); ++it)
	{
		if (subscriber.get() == NULL)
			continue;

		if (subscriber->getUuid() == (*it)->getUuid())
		{
			subscriber_pos = subscriber_idx;
			break;
		}
		subscriber_idx ++;
	}
	if (subscriber_pos < m_activeSubscribers.size())
	{
		m_activeSubscribers[subscriber_pos] = false;
		return true;
	}

	return false;
}

bool ZyROSConnectorWorkerThread::addTopicPublisher(boost::shared_ptr<ZyROSPublisher>& publisher)
{
    boost::mutex::scoped_lock lock(m_mutex);
    for (std::vector<boost::shared_ptr<ZyROSPublisher> >::const_iterator it = m_topicPublishers.begin(); it != m_topicPublishers.end(); ++it)
    {
        if (publisher->getUuid() == (*it)->getUuid())
        {
            return false;
        }
    }

    m_topicPublishers.push_back(boost::move(publisher));
    m_activePublishers.push_back(true);
    return true;
}

bool ZyROSConnectorWorkerThread::removeTopicPublisher(boost::shared_ptr<ZyROSPublisher>& publisher)
{
    boost::mutex::scoped_lock lock(m_mutex);

    size_t publisher_pos = -1, publisher_idx = 0;
    for (std::vector<boost::shared_ptr<ZyROSPublisher> >::iterator it = m_topicPublishers.begin(); it != m_topicPublishers.end(); ++it)
    {
        if (publisher.get() == NULL)
            continue;

        if (publisher->getUuid() == (*it)->getUuid())
        {
            publisher_pos = publisher_idx;
            break;
        }
        // TODO / NOTE: shouldn't publisher_idx be incremented somewhere?
    }
    if (publisher_pos >= 0 && publisher_pos < m_activePublishers.size())
    {
        m_activePublishers[publisher_pos] = false;
        return true;
    }

    return false;
}

size_t ZyROSConnectorWorkerThread::getNumTopicListeners() const
{
    return m_topicSubscribers.size();
}

size_t ZyROSConnectorWorkerThread::getNumTopicPublishers() const
{
    return m_topicPublishers.size();
}

void ZyROSConnectorWorkerThread::main()
{
  msg_info() << "Entering main function of ZyROSConnectorWorkerThread" << sendl;
	// read and store current thread id
	m_id = get_current_thread_id();

	// signal that the thread is running
	signal_state(running);

	// perform on-start custom action
	on_start();

	if (m_start_paused)
	{
		m_request = rq_idle;
		signal_state(paused);
	}

	// can throw const boost::thread_interrupted
	// if interrupt() was call in any interrupt
	// point
	try
	{
		(m_rosConnector->*m_func)();
	}
	catch (const boost::thread_interrupted& ex)
	{
		SOFA_UNUSED(ex);
		std::cerr << "WARNING: Thread " << m_name << ": Caught boost::thread_interrupted" << std::endl;
	}

	// update state
	signal_state(completed);

	// perform on-exit custom action
	// after the state was updated
	on_exit();
	// clear id
	m_id = INVALID_THREAD_ID;
}
