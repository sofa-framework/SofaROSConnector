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
        std::cout << "(ZySimulationAnalyzerHandler::setROSConnectionManagerByContext) Found the ZyROSSimulationAnalyzer " << simulationAnalyzerPrivate->simulationAnalyzer->getName() << std::endl;
        return true;
    }
    else
    {
        std::cout << "(ZySimulationAnalyzerHandler::setROSConnectionManagerByContext) WARNING: Could not find a ZyROSSimulationAnalyzer, cannot publish messages." << std::endl;
        return false;
    }    
}

void ZySimulationAnalyzerHandler::analyzeSimulationStateAndPublishResult()
{
    simulationAnalyzerPrivate->simulationAnalyzer->analyzeSimulationStateAndPublishResult();
}
