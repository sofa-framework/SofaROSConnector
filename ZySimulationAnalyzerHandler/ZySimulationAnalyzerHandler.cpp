#include "ZySimulationAnalyzerHandler.h"

#include <ZyROSSimulationAnalyzer.h>

using namespace Zyklio::SimulationAnalysis;

class Zyklio::SimulationAnalysis::ZySimulationAnalyzerPrivate
{
    public:
        ZyROSSimulationAnalyzer* simulationAnalyzer;
};

ZySimulationAnalyzerHandler::ZySimulationAnalyzerHandler()
    : simulationAnalyzerPrivate(NULL)
{
    simulationAnalyzerPrivate = new ZySimulationAnalyzerPrivate();
}

ZySimulationAnalyzerHandler::~ZySimulationAnalyzerHandler()
{
    delete simulationAnalyzerPrivate;
}

bool ZySimulationAnalyzerHandler::setSimAnalyzerByContext(sofa::core::objectmodel::BaseContext* cntxt)
{
    simulationAnalyzerPrivate->simulationAnalyzer = cntxt->getRootContext()->get< ZyROSSimulationAnalyzer >();

    if (simulationAnalyzerPrivate->simulationAnalyzer)
    {
        msg_info("ZySimulationAnalyzerHandler") << "Found ZyROSSimulationAnalyzer instance: " << simulationAnalyzerPrivate->simulationAnalyzer->getName();
        return true;
    }
    else
    {
        msg_warning("ZySimulationAnalyzerHandler") << "Could not find a ZyROSSimulationAnalyzer instance, cannot publish messages.";
        return false;
    }    
}

void ZySimulationAnalyzerHandler::analyzeSimulationStateAndPublishResult()
{
    simulationAnalyzerPrivate->simulationAnalyzer->analyzeSimulationStateAndPublishResult();
}
