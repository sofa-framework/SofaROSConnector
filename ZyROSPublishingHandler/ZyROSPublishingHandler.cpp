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
        std::cout << "(ZyROSPublishingHandler::setROSConnectionManagerByContext) Found the ROSConnectionManager " << connectionManagerPrivate->connectionManager->getName() << std::endl;
        return true;
    }
    std::cout << "(ZyROSPublishingHandler::setROSConnectionManagerByContext) WARNING: Could not find a ROSConnectionManager, cannot publish messages." << std::endl;
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
