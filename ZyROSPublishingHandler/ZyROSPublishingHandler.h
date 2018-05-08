#ifndef TRU_SOFA_PUBLISHINGHANDLER_H
#define TRU_SOFA_PUBLISHINGHANDLER_H

#ifdef _WIN32
// Ensure that Winsock2.h is included before Windows.h, which can get
// pulled in by anybody (e.g., Boost).
#include <Winsock2.h>
#endif

#include "init_ZyROSPublishingHandler.h"
#include "ZyROSConnectorTopicPublisher.h"

#include <sofa/core/objectmodel/BaseContext.h>
#include "ZyROSConnectorTopicSubscriber.h"

namespace Zyklio
{
    namespace ROSPublishing
    {
        class ZyROSConnectionManagerPrivate;

        // TODO: Maybe rename this component so that it does not refer to "publishing" anymore - or even better, find a different solution to the ROSConnector <-> ConnectionManager linking problem.

        /**
         * \brief This is a helper class to register arbitrary TruRosPublishers with a TruRosConnector managed
         * by a ZyROSConnectionManager.
         *
         * When an instance of a class that inherits from ZyROSPublisher (for example ZyROSFloat32Publisher) is
         * needed, instantiate an ZyROSPublishingHandler, call the setROSConnectionManagerByContext method (CAREFUL!
         * When this is called, the ZyROSConnectionManager in the scene graph must already be instantiated, so
         * a constructor or an init() method are bad places to call setROSConnectionManagerByContext) and then
         * create the ZyROSPublisher instance using the getROSNodeHandle method. Finally, register the publisher with
         * the ROS connector, using the registerPublisher method;
         */
        class SOFA_ZY_ROS_PUBLISHINGHANDLER_API ZyROSPublishingHandler
        {
        public:
            ZyROSPublishingHandler();
            ~ZyROSPublishingHandler();

            /**
            * \brief
            *
            * (CAREFUL! When this is called, the ZyROSConnectionManager in the scene graph must already
            * be instantiated, so a constructor or an init() method are bad places to call setROSConnectionManagerByContext)
            *
            * \param cntxt
            */
            bool setROSConnectionManagerByContext(sofa::core::objectmodel::BaseContext* cntxt);

            template <class MessageType>
            void registerSubscriber(boost::shared_ptr<ROSConnector::ZyROSConnectorTopicSubscriber<MessageType>> sub)
            {
                boost::shared_ptr<ROSConnector::ZyROSListener> subscriber = boost::dynamic_pointer_cast<ROSConnector::ZyROSListener>(sub);
                registerSubscriber(subscriber);
            }

            void registerPublisher(boost::shared_ptr<ROSConnector::ZyROSPublisher>& pub);
            void registerSubscriber(boost::shared_ptr<ROSConnector::ZyROSListener>& sub);

            void registerPublisher(ROSConnector::ZyROSPublisher* pub);
            void registerSubscriber(ROSConnector::ZyROSListener* sub);

            ros::NodeHandlePtr getROSNodeHandle();

            std::vector< boost::shared_ptr<ROSConnector::ZyROSListener> >* getTopicListeners();

            template <class MessageType>
            std::vector< boost::shared_ptr< ROSConnector::ZyROSConnectorTopicSubscriber<MessageType> > > getSubscribers()
            {
                std::string typeString = ros::message_traits::DataType<MessageType>::value();
                std::vector< boost::shared_ptr< ROSConnector::ZyROSConnectorTopicSubscriber<MessageType> > > retVec;

                std::vector< boost::shared_ptr<ROSConnector::ZyROSListener> >* listeners = getTopicListeners();
                for (std::vector< boost::shared_ptr<ROSConnector::ZyROSListener> >::iterator
                    it = listeners->begin();
                    it != listeners->end();
                    it++)
                {
                    if ((*it)->getMessageType().compare(typeString) == 0)
                    {
                        boost::shared_ptr< ROSConnector::ZyROSConnectorTopicSubscriber<MessageType> > subPnt =
                            boost::dynamic_pointer_cast<ROSConnector::ZyROSConnectorTopicSubscriber<MessageType>>((*it));
                        retVec.push_back(subPnt);
                    }
                }

                return retVec;
            }

            template <class MessageType>
            boost::shared_ptr< ROSConnector::ZyROSConnectorTopicSubscriber<MessageType> > getSubscriber(std::string topic)
            {
                std::vector< boost::shared_ptr< ROSConnector::ZyROSConnectorTopicSubscriber<MessageType> > > subs = getSubscribers<MessageType>();

                for (
#ifndef _WIN32
                    typename
#endif
                    std::vector< boost::shared_ptr< ROSConnector::ZyROSConnectorTopicSubscriber<MessageType> > >::iterator
                    it = subs.begin();
                    it != subs.end();
                    it++)
                {
                    if ((*it)->getTopic().compare(topic) == 0)
                    {
                        return (*it);
                    }
                }

                return boost::shared_ptr< ROSConnector::ZyROSConnectorTopicSubscriber<MessageType> >();
            }


        private:
            ZyROSConnectionManagerPrivate* connectionManagerPrivate;
        };
    }
}

#endif // TRU_SOFA_PUBLISHINGHANDLER_H
