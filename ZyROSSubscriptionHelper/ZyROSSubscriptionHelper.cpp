#include "ZyROSSubscriptionHelper.h"

#include <ZyROSConnectionManager.h>
#include <ZyROSSimulationAnalyzer.h>

using namespace Zyklio::ROSConnector;

class Zyklio::ROSConnector::ZyROSSimulationAnalyzerPrivate
{
    public:
        ROSConnectionManager::ZyROSConnectionManager* connectionManager;
        SimulationAnalysis::ZyROSSimulationAnalyzer* simulationAnalyzer;
};

ZyROSSubscriptionHelper::ZyROSSubscriptionHelper()
    : simulationAnalyzerPrivate(NULL)
{
    simulationAnalyzerPrivate = new ZyROSSimulationAnalyzerPrivate();
}

bool ZyROSSubscriptionHelper::setROSConnectionManagerByContext(sofa::core::objectmodel::BaseContext* cntxt)
{
    simulationAnalyzerPrivate->connectionManager = cntxt->getRootContext()->get< ROSConnectionManager::ZyROSConnectionManager >();
    simulationAnalyzerPrivate->simulationAnalyzer = cntxt->getRootContext()->get< SimulationAnalysis::ZyROSSimulationAnalyzer >();

    bool everythingOK = false;
    if (simulationAnalyzerPrivate->connectionManager)
    {
        std::cout << "(ZyROSSubscriptionHelper::setROSConnectionManagerByContext) Found the ROSConnectionManager " << simulationAnalyzerPrivate->connectionManager->getName() << std::endl;
        everythingOK = true;
    }
    else
    {
        std::cout << "(ZyROSSubscriptionHelper::setROSConnectionManagerByContext) WARNING: Could not find a ROSConnectionManager, cannot publish messages." << std::endl;
        everythingOK = false;
    }

    if (simulationAnalyzerPrivate->simulationAnalyzer)
    {
        std::cout << "(ZyROSSubscriptionHelper::setROSConnectionManagerByContext) Found the ZyROSSimulationAnalyzer " << simulationAnalyzerPrivate->simulationAnalyzer->getName() << std::endl;
        everythingOK = everythingOK && true;
    }
    else
    {
        std::cout << "(ZyROSSubscriptionHelper::setROSConnectionManagerByContext) WARNING: Could not find a ZyROSSimulationAnalyzer, cannot set anything there." << std::endl;
        everythingOK = everythingOK && false;
    }

    return everythingOK;
}

void ZyROSSubscriptionHelper::setLinValueInAnalyzer(float lin)
{
    if (simulationAnalyzerPrivate->simulationAnalyzer)
    {
        simulationAnalyzerPrivate->simulationAnalyzer->setLinInput(lin);
    }
}

void ZyROSSubscriptionHelper::setRotValueInAnalyzer(float rot)
{
    if (simulationAnalyzerPrivate->simulationAnalyzer)
    {
        simulationAnalyzerPrivate->simulationAnalyzer->setRotInput(rot);
    }
}

ZyROSSubscriptionHelper::~ZyROSSubscriptionHelper()
{
    delete simulationAnalyzerPrivate;
}

ros::NodeHandlePtr ZyROSSubscriptionHelper::getROSNodeHandle()
{
    return simulationAnalyzerPrivate->connectionManager->getRosNodeHandle();
}
