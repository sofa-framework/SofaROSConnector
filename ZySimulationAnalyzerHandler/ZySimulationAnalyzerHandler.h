#ifndef SOFA_ZY_SIMULATIONANALYSISHANDLER_H
#define SOFA_ZY_SIMULATIONANALYSISHANDLER_H

#ifdef _WIN32
// Ensure that Winsock2.h is included before Windows.h, which can get
// pulled in by anybody (e.g., Boost).
#include <Winsock2.h>
#endif

#include "init_ZySimulationAnalyzerHandler.h"

#include <sofa/core/objectmodel/BaseContext.h>
#include <sofa/simulation/MechanicalOperations.h>
#include <sofa/simulation/VectorOperations.h>

namespace Zyklio
{
    namespace SimulationAnalysis
    {
        class ZySimulationAnalyzerPrivate;

        /**
         * \brief This is a helper class to help with a ZyROSSimulationAnalyzer.
         */
        class SOFA_ZY_SIMULATION_ANALYSIS_HANDLER_API ZySimulationAnalyzerHandler
		{
            public:
                ZySimulationAnalyzerHandler();
                ~ZySimulationAnalyzerHandler();

                /**
                * \brief 
                * 
                * (CAREFUL! When this is called, the ZySimulationAnalyzer in the scene graph must already
                * be instantiated, so a constructor or an init() method are bad places to call setVelocityApproximatorByContext)
                * 
                * \param cntxt 
                */
                bool setSimAnalyzerByContext(sofa::core::objectmodel::BaseContext* cntxt);
                void analyzeSimulationStateAndPublishResult();
                
            private:
                ZySimulationAnalyzerPrivate* simulationAnalyzerPrivate;
		};
	}
}

#endif // SOFA_ZY_SIMULATIONANALYSISHANDLER_H
