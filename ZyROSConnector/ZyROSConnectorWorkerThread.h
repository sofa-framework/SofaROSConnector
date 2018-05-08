#ifndef TRUROSCONNECTOR_WORKER_THREAD_H
#define TRUROSCONNECTOR_WORKER_THREAD_H

#ifdef _WIN32
// Ensure that Winsock2.h is included before Windows.h, which can get
// pulled in by anybody (e.g., Boost).
#include <Winsock2.h>
#endif

#include "init_ZyROSConnector.h"
#include <ZyMultiThreading/WorkerThread_SingleTask.h>

#include "ZyROSConnector.h"
#include "ZyROSConnectorTopicPublisher.h"

namespace Zyklio
{
	namespace ROSConnector
	{
        class SOFA_ZY_ROS_CONNECTOR_API ZyROSConnectorWorkerThread : public Zyklio::MultiThreading::WorkerThread_SingleTask
		{
			public:
                ZyROSConnectorWorkerThread(ZyROSConnector*);
                ~ZyROSConnectorWorkerThread();

                bool addTopicListener(const boost::shared_ptr<ZyROSListener> &);
                bool removeTopicListener(boost::shared_ptr<ZyROSListener>&);

                bool addTopicPublisher(boost::shared_ptr<ZyROSPublisher>&);
                bool removeTopicPublisher(boost::shared_ptr<ZyROSPublisher>&);

                size_t getNumTopicListeners() const;
                size_t getNumTopicPublishers() const;

			protected:
				void main();

			private:
				friend class ZyROSConnector;

				void (ZyROSConnector::*m_func) (void);
				ZyROSConnector* m_rosConnector;

                boost::mutex m_mutex;
                std::vector<boost::shared_ptr<ZyROSListener> > m_topicSubscribers;
                std::vector<bool> m_activeSubscribers;
                std::vector<boost::shared_ptr<ZyROSPublisher> > m_topicPublishers;
                std::vector<bool> m_activePublishers;
		};
	}
}

#endif // TRUROSCONNECTOR_WORKER_THREAD_H
