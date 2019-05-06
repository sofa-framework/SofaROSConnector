#include "ZyROSPublishingHandler.h"

#include <ZyROSConnectionManager.h>

using namespace Zyklio::ROSPublishing;

class Zyklio::ROSPublishing::ZyROSConnectionManagerPrivate
{
public:
    ROSConnectionManager::ZyROSConnectionManager* connectionManager;
};

ZyROSPublishingHandler::ZyROSPublishingHandler()
    : connectionManagerPrivate(NULL)
{
    connectionManagerPrivate = new ZyROSConnectionManagerPrivate();
}

bool ZyROSPublishingHandler::setROSConnectionManagerByContext(sofa::core::objectmodel::BaseContext* cntxt)
{
    if (!connectionManagerPrivate->connectionManager)
    {
        connectionManagerPrivate->connectionManager = cntxt->getRootContext()->get< ROSConnectionManager::ZyROSConnectionManager>();
    }

    if (connectionManagerPrivate->connectionManager)
    {
        msg_info("ZyROSPublishingHandler") << "Found a ROSConnectionManager: " << connectionManagerPrivate->connectionManager->getName();
        return true;
    }

    msg_warning("ZyROSPublishingHandler") << "Could not find a ROSConnectionManager, cannot publish messages.";
    return false;
}

ZyROSPublishingHandler::~ZyROSPublishingHandler()
{
    delete connectionManagerPrivate;
}

void ZyROSPublishingHandler::registerPublisher(boost::shared_ptr<ZyROSPublisher>& pub)
{
    if (connectionManagerPrivate->connectionManager)
    {
        connectionManagerPrivate->connectionManager->addPublisher(pub);
    }
    else
    {
        std::cout << "(ZyROSPublishingHandler::registerPublisher) WARNING: Could not find a ROSConnectionManager, cannot publish messages." << std::endl;
    }
}

void ZyROSPublishingHandler::registerSubscriber(boost::shared_ptr<ZyROSListener>& sub)
{
    if (connectionManagerPrivate->connectionManager)
    {
        connectionManagerPrivate->connectionManager->addSubscriber(sub);
    }
    else
    {
        std::cout << "(ZyROSPublishingHandler::registerSubscriber) WARNING: Could not find a ROSConnectionManager, cannot publish messages." << std::endl;
    }
}

void ZyROSPublishingHandler::registerPublisher(ZyROSPublisher* pub)
{
    boost::shared_ptr<ZyROSPublisher> publisher(pub);
    registerPublisher(publisher);
}

void ZyROSPublishingHandler::registerSubscriber(ZyROSListener* sub)
{
    boost::shared_ptr<ZyROSListener> subscriber(sub);
    registerSubscriber(subscriber);
}

ros::NodeHandlePtr ZyROSPublishingHandler::getROSNodeHandle()
{
    if (connectionManagerPrivate->connectionManager)
    {
        return connectionManagerPrivate->connectionManager->getRosNodeHandle();
    }
    return ros::NodeHandlePtr();
}

std::vector<boost::shared_ptr<ZyROSListener>>* ZyROSPublishingHandler::getTopicListeners()
{
    if (connectionManagerPrivate->connectionManager)
    {
        return connectionManagerPrivate->connectionManager->getTopicListeners();
    }
    return NULL;
}
