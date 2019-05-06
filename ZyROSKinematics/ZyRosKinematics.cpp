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
#include "ZyRosKinematics.h"

#include <SofaConstraint/FrictionContact.h>
#include "sofa/core/ObjectFactory.h"

namespace Zyklio
{
    namespace ROSKinematics
    {

        using namespace sofa;

        SOFA_DECL_CLASS(ZyROSKinematics)

        int ZyROSKinematicsClass = core::RegisterObject("Control a kinematics chain with a ROS topic.")
        .add< ZyROSKinematics >()
        ;

        ZyROSKinematics::ZyROSKinematics()
            : topicName(initData(&topicName, "topicName", "The topic which delivers the joint states for the kinematics chain."))
            , arbitraryControllerPath(initData(&arbitraryControllerPath, "arbitraryControllerPath", "Path to the ArbitraryController element in the scene graph which controls the kinematics chain."))
            , arbitraryControllerPnt(NULL)
            , velocityApproximationHandler()
            , velocityApproximationFound(false)
        {
            theTime = boost::posix_time::microsec_clock::local_time();
        }

        void ZyROSKinematics::init()
        {
            Inherit::init();
        }

        void ZyROSKinematics::bwdInit()
        {
            msg_info("ZyROSKinematics") << "bwdInit()";
            ROSPublishing::ZyROSPublishingHandler publishingHandler;
            bool connectionManagerFound = publishingHandler.setROSConnectionManagerByContext(getContext());

            if (connectionManagerFound)
            {
                msg_info("ZyROSKinematics") << "Found ZyROSConnectionManagerInstance, subscribing to JointState topic: " << topicName.getValue();
                jointStateSubscriber = boost::shared_ptr< ROSConnector::ZyROSConnectorTopicSubscriber<sensor_msgs::JointState> >(
                    new ROSConnector::ZyROSConnectorTopicSubscriber<sensor_msgs::JointState>(publishingHandler.getROSNodeHandle(), topicName.getValue(), 50, false)
                    );

                subscriberConnection = jointStateSubscriber->getSignal().connect(boost::bind(&ZyROSKinematics::handleJointStateMessage, this));

                publishingHandler.registerSubscriber<sensor_msgs::JointState>(jointStateSubscriber);
            }

            arbitraryControllerPnt = getContext()->getRootContext()->get<component::controller::ArbitraryController>(arbitraryControllerPath.getValue());

            if (arbitraryControllerPnt)
            {
                msg_info("ZyROSKinematics") << "Found the ArbitraryController " << arbitraryControllerPnt->getName();
            }
            else
            {
                msg_warning("ZyROSKinematics") << "Could not find the ArbitraryController in the path \"" << arbitraryControllerPath.getValue() << "\"";
            }

            // TODO: Move this to the velocity approximation itself
            velocityApproximationFound = velocityApproximationHandler.setVelocityApproximatorByContext(getContext());
            if (velocityApproximationFound)
            {
                msg_info("ZyROSKinematics") << "Found a ZyVelocityApproximator.";
            }

            Inherit::bwdInit();
        }

        void ZyROSKinematics::reinit()
        {
            init();
            bwdInit();
        }

        void ZyROSKinematics::handleJointStateMessage()
        {
            msg_info("ZyROSKinematics") << "Received a new JointState message.";
            const sensor_msgs::JointState& msg = jointStateSubscriber->getLatestMessage();

            if (velocityApproximationFound)
            {
                sofa::helper::vector<std::pair <double, std::string>> jntDtVec;
                for (size_t k = 0; k < msg.name.size(); k++)
                {
                    std::pair <double, std::string> jntDt(msg.position[k], msg.name[k]);
                    jntDtVec.push_back(jntDt);
                }

                velocityApproximationHandler.pushJointMsg(std::pair<std::pair<unsigned int, double>, sofa::helper::vector< std::pair <double, std::string> > >(std::pair<unsigned int, double>(msg.header.seq, msg.header.stamp.toSec()), jntDtVec));
            }

            updateJointState(msg);
        }

        void ZyROSKinematics::updateJointState(const sensor_msgs::JointState& joint_state)
        {
            boost::posix_time::ptime curTime = boost::posix_time::microsec_clock::local_time();
            boost::posix_time::time_duration dur = (curTime - theTime);
            if (dur.total_milliseconds() > 100)
            {
                msg_info("ZyROSKinematics") << "Setting joint values: " << joint_state.name.size();
                for (size_t k = 0; k < joint_state.name.size(); k++)
                {
                    std::cout << joint_state.name[k] << ";";
                    if (arbitraryControllerPnt != NULL)
                        arbitraryControllerPnt->setRotValueRadByName(joint_state.position[k], joint_state.name[k]);
                }

                theTime = boost::posix_time::microsec_clock::local_time();
            }
        }
    } // namespace ROSKinematics

} // namespace Zyklio
