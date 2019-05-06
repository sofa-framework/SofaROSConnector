/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 RC 1        *
*                (c) 2006-2011 MGH, INRIA, USTL, UJF, CNRS                    *
*                                                                             *
* This library is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This library is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this library; if not, write to the Free Software Foundation,     *
* Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA.          *
*******************************************************************************
*                               SOFA :: Modules                               *
*                                                                             *
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#ifndef ZY_SIMULATIONANALYZER_H
#define ZY_SIMULATIONANALYZER_H

#ifdef _WIN32
// Ensure that Winsock2.h is included before Windows.h, which can get
// pulled in by anybody (e.g., Boost).
#include <Winsock2.h>
#endif

#include "initZyROSSimulationAnalyzer.h"

#include <sofa/core/objectmodel/Base.h>
#include <sofa/core/objectmodel/BaseObject.h>

#include <sofa/simulation/MechanicalOperations.h>
#include <sofa/simulation/VectorOperations.h>

#include <ZyROSPublishingHandler.h>
#include "ZyROSTopicPublisher_Float32MultiArray.h"

#include <boost/date_time/posix_time/posix_time.hpp> 

namespace Zyklio
{
    namespace SimulationAnalysis
    {
        using namespace sofa;

        class SOFA_ZY_SIMULATION_ANALYSIS_API ZyROSSimulationAnalyzer : public /*virtual*/ core::objectmodel::BaseObject
        {
        public:
            typedef core::objectmodel::BaseObject Inherit;

            SOFA_CLASS(ZyROSSimulationAnalyzer, core::objectmodel::BaseObject);

            typedef sofa::core::objectmodel::BaseContext BaseContext;
            typedef sofa::core::objectmodel::BaseObjectDescription BaseObjectDescription;

        public:

            ZyROSSimulationAnalyzer();
            ~ZyROSSimulationAnalyzer() {}

            void init();
            void bwdInit();
            void reinit();

            void analyzeSimulationStateAndPublishResult();

            void setLinInput(float lin) { linInput = lin; }
            void setRotInput(float rot) { rotInput = rot; }

            Zyklio::ROSPublishing::ZyROSPublishingHandler publishingHandler;
            Zyklio::ROSConnector::ZyROSFloat32MultiArrayPublisher* simResultPublisher;
            bool connectionManagerFound;

            //boost::posix_time::ptime theTime;
            float timeAtStart;

            Data<std::string> targetName; // how is the object called, that is supposed to be analyzed?
            float linInput, rotInput;
            defaulttype::Vec3d oldPos;

            Data<double> targetMinHeight; // what height does the object need to reach to count as success?
            Data<double> targetMovementTolerance; // how much is the object allowed to move in a gripped position?
            Data<unsigned int> targetIterations; // how many iterations does it need to stand still at that height to count as success?
            unsigned int targetCounter;

        };

    } // namespace SimulationAnalysis

} // namespace Zyklio

#endif //ZY_SIMULATIONANALYZER_H
