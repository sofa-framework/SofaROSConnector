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
#ifndef Zyklio_ZyROSKinematics_H
#define Zyklio_ZyROSKinematics_H

#ifdef _WIN32
// Ensure that Winsock2.h is included before Windows.h, which can get
// pulled in by anybody (e.g., Boost).
#include <Winsock2.h>
#endif

#include "initZyRosKinematics.h"

#include <sofa/core/objectmodel/Base.h>
#include <sofa/core/objectmodel/BaseObject.h>

#include <ZyROSConnector.h>
#include <ZyROSPublishingHandler.h>

#include <ArbitraryController.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include "ZyVelocityApproximationHandler.h"

namespace Zyklio
{
    namespace ROSKinematics
    {
        using namespace sofa;

        /**
         * \brief TODO: Document this
         */
        class SOFA_ZY_ROS_KINEMATICS_API ZyROSKinematics : public core::objectmodel::BaseObject
        {
        public:
            typedef core::objectmodel::BaseObject Inherit;

            SOFA_CLASS(ZyROSKinematics, core::objectmodel::BaseObject);

            typedef sofa::core::objectmodel::BaseContext BaseContext;
            typedef sofa::core::objectmodel::BaseObjectDescription BaseObjectDescription;
            
            ZyROSKinematics();
            ~ZyROSKinematics() {}

            void init();
            void bwdInit();
            void reinit();

        private:
            Data<std::string> topicName;
            Data<std::string> arbitraryControllerPath;

            boost::shared_ptr< Zyklio::ROSConnector::ZyROSConnectorTopicSubscriber<sensor_msgs::JointState> > jointStateSubscriber;
            boost::signals2::connection subscriberConnection;

            component::controller::ArbitraryController* arbitraryControllerPnt;
            VelocityApproximation::ZyROSVelocityApproximationHandler velocityApproximationHandler;
            bool velocityApproximationFound;
            boost::posix_time::ptime theTime;

            void handleJointStateMessage();
            void updateJointState(const sensor_msgs::JointState& joint_state);
        };

    } // namespace VelocityApproximation

} // namespace Zyklio

#endif //Zyklio_ZyROSKinematics_H
