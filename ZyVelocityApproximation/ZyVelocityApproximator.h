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
#ifndef TRUPHYSICS_TRUVELOCITYAPPROXIMATION_H
#define TRUPHYSICS_TRUVELOCITYAPPROXIMATION_H

#include "initZyVelocityApproximation.h"

#include <sofa/core/objectmodel/Base.h>
#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/core/objectmodel/BaseObjectDescription.h>
#include <sofa/core/objectmodel/BaseContext.h>
#include <sofa/simulation/Node.h>

#include <SofaBaseMechanics/MechanicalObject.h>
#include <sofa/simulation/MechanicalOperations.h>
#include <sofa/simulation/VectorOperations.h>

#include <sofa/core/behavior/MultiVec.h>

#include <sofa/defaulttype/Vec3Types.h>

#include <boost/circular_buffer.hpp>
#include <ArbitraryController.h>
#include <chrono>

#define ROSTIMESTEPS_USED_FOR_AVERAGE 100

// 

namespace Zyklio
{
    namespace VelocityApproximation
    {
        using namespace sofa;
        using namespace sofa::component;
        using namespace sofa::component::controller;

        /**
         * \brief New velocity approximation implementation:
         * 
         * The general idea behind the implementation is the same as before. The velocity
         * approximation class collects joint states messages. Once every simulation step,
         * the approximation is called, which then replays all the collected messages and
         * approximates the velocity.
         * 
         * The main difference is that now the joint states subscriber has its own
         * implementation, so the messages are given to the velocity approximation from
         * there.
         * 
         * Also, similar to the ROS connector, the velocity approximation is now split
         * into :
         *  - a plugin, which provides the scene graph component
         *  - a lib called "handler" which makes it possible for other plugins to
         *    interact with the velocity approximation, without the need to link against
         *    the scene graph plugin.
         */
        class SOFA_TRUVELOCITYAPPROXIMATION_API TruVelocityApproximator : public /*virtual*/ core::objectmodel::BaseObject
        {
        public:
            typedef core::objectmodel::BaseObject Inherit;

            SOFA_CLASS(TruVelocityApproximator, core::objectmodel::BaseObject);

            typedef sofa::core::objectmodel::BaseContext BaseContext;
            typedef sofa::core::objectmodel::BaseObjectDescription BaseObjectDescription;

        public:

            TruVelocityApproximator();
            ~TruVelocityApproximator() {}

            void moveObjectToPosition(simulation::common::MechanicalOperations& mop, sofa::core::behavior::MultiVecCoord& pos/*, sofa::core::behavior::MultiVecCoord& freePos*/);
            void setObjectPosition(defaulttype::RigidTypes::VecCoord newPosition) { currentPosition = newPosition; positionChange = true; }

            //void approximateVelocity();
            void approximateVelocity(simulation::common::MechanicalOperations& mop, sofa::core::behavior::MultiVecCoord& pos);

            void init();
            void bwdInit();
            void reinit();

            Data<std::string> objectNodePath;    // TODO: replace this with the proper SOFA way to do this
            Data<std::string> collisionNodePath; // TODO: replace this with the proper SOFA way to do this
                            
            Data<bool> rosControl; // switches if a joint is controlled by a ros topic or the keyboard control
                
            // used to differentiate between different joint_states message publishers
            // only joint_states messages in which one joint has this name are processed
            Data< std::string > jointNameInJointStatesMsg;

            //Data<unsigned int> selectedVertex; // select the vertex for which the velocity approximation is done
            Data< helper::vector< unsigned int > > selectedVertices;

            //Data< sofa::helper::vector<double> > currentVelocityData; // current time-step; velocity vector (x/y/z); velocity magnitude; position vector (x/y/z)

            // Data members used to give easy access to the velocity approximation from a python script
            // Data< sofa::helper::vector<sofa::helper::vector<double>> > currentVelocityData; // vector of vectors which each contain: current time-step; velocity vector (x/y/z); velocity magnitude; position vector (x/y/z)
            //Data< sofa::helper::vector< velocityData > > currentVelocityData; // vector of arrays which each contain: current time-step; velocity vector (x/y/z); velocity magnitude; position vector (x/y/z)
            //Data< identificationData > currentVelocityIdentificationData; // contains: unique identifier (is only set during init); mesh name (is only set during init); selected vertex
               
            typedef sofa::helper::fixed_array<double, 8> velocityData; // contains : current time - step; velocity vector(x / y / z); velocity magnitude; position vector(x / y / z)
            typedef sofa::helper::fixed_array<std::string, 5> identificationData; // contains: unique identifier(is only set during init); mesh name(is only set during init); selected vertex
            //typedef sofa::helper::vector< std::pair< identificationData, velocityData> > velocityApproximationData;
            //Data < velocityApproximationData > currentVelocityApproximationData;
            typedef sofa::helper::fixed_array<std::string, 13> currentVelocityApproximationDataType;
            Data < sofa::helper::vector< currentVelocityApproximationDataType > > currentVelocityApproximationData; // Unfortunately, using the type "velocityApproximationData" is not possible with Data 
                    
#ifdef TRUPHYSICS_DEMO
            template<class T>
            static bool canCreate(T*& obj, core::objectmodel::BaseContext* context, core::objectmodel::BaseObjectDescription* arg)
            {
                std::string sceneHash = sofa::simulation::getSimulation()->getSceneHash();

                sofa::helper::hashCheckHelper hch;
                if (hch.checkHashCorrectness(sceneHash))
                {
                    return Inherit::canCreate(obj, context, arg);
                }

                std::cout << "This plugin can only be used with the Zyklio demo and the correct demo scene." << std::endl;
                return false;
            }
#endif 
                
            typedef std::pair <double, std::string> jointData;  // combines a joint value with its name
            //typedef std::pair<unsigned int, sofa::helper::vector<jointData> > jointMsg;  // combines all joint values and names in a joint_states message with that message's seq number
            typedef std::pair<std::pair<unsigned int,double>, sofa::helper::vector<jointData> > jointMsg;  // combines all joint values and names in a joint_states message with that message's seq number and timestamp
                
            /*defaulttype::Vec3Types::VecDeriv getVelocityApprox() { return approxVelocityVec3; };
            defaulttype::Vec3Types::Deriv getVertexVelocityApprox(unsigned int i) { return approxVelocityVec3[i]; };*/ // currently not looking at max velocity
            const std::string& getJointNameInJointStatesMsg() { return jointNameInJointStatesMsg.getValue(); }
            //void setNewRosTimeAndStamp(double newTime, unsigned int newStamp);
            void pushJointMsg(jointMsg theMsg);
                
        private:
            sofa::component::container::MechanicalObject<defaulttype::RigidTypes>* objectState;
            defaulttype::RigidTypes::VecCoord currentPosition; // for setting the object position

            //defaulttype::RigidTypes::VecCoord oldPosRigid; // for approximating the velocity
            //defaulttype::RigidTypes::VecDeriv approxVelocityRigid; // for approximating the velocity

            //defaulttype::Vec3Types::VecCoord oldPosVec3; // for approximating the velocity
            std::vector< defaulttype::Vec3Types::VecCoord > oldPosVec3Vec; // for approximating the velocity

            //defaulttype::Vec3Types::VecDeriv approxVelocityVec3; // for approximating the velocity
            //double maxVelocitySqVec3;
            //double maxVelocityTimeVec3;
            //unsigned int maxVelocityIndexVec3; // currently not looking at max velocity

            bool positionChange;
            double lastResetTime;

            std::string identificationString/*, meshName*/;
            std::vector < std::string > meshNameVec;
            boost::circular_buffer<jointMsg>::iterator oldIt;

            // currently not used 
            //boost::circular_buffer<std::pair<double, unsigned int>> rosTimeAndStamps;
                
            // rosJointMsgs 
            // Contains all joint_message data of the last n (currently n=80) messages.
            // Is filled in ZyROSConnector::updateJointState, currently.
            // jointMsg: pair (<ros message data>, <joint vector>),
            //      ros message data: pair(ros sequence (uint), ros timestep (double))
            //      joint vector : vector of <joint data>
            //          joint data : pair (joint value (double), joint name (string))
            boost::circular_buffer<jointMsg> rosJointMsgs;
            boost::circular_buffer<double> rosTimeSteps;
            std::vector<jointMsg> missedRosJointMsgs; // used during replay of missed messages
                
            unsigned int currentRosStamp, oldRosStamp;
            double currentRosTime, oldRosTime;

            //double selectedVertex_x, selectedVertex_y, selectedVertex_z;

            unsigned int lastPushedStamp;
            double lastPushedMsgTime;

            unsigned int /*vertexTornado,*/ indexTornado, testRunIndexTornado;
            //double vertexTornado_x, vertexTornado_y, vertexTornado_z;
            std::vector<double> vertexTornadoVec_x, vertexTornadoVec_y, vertexTornadoVec_z;
            //std::string tornadoObjectName;
            std::vector<std::string> tornadoObjectNameVec;
            bool calledByTornado;
            bool runVelocityApproximation;
            Data<sofa::helper::vector < std::string > > selectedVertexInfo;

            //sofa::component::container::MechanicalObject<defaulttype::Vec3Types>* collisionState;
            std::vector < sofa::component::container::MechanicalObject<defaulttype::Vec3Types>* > collisionStateVec;
            //sofa::helper::vector<std::string> collisionModelNames;

            ArbitraryController* arbitraryController;

            currentVelocityApproximationDataType createOutputArray(const std::pair< identificationData, velocityData>& velocityDataPair);
            std::vector<double> generateVertexVector(char* vertexInput);
            std::vector<std::string> generateObjectNameVector(char* vertexInput);

            double adjustTime(double time);

            //std::chrono::time_point<std::chrono::system_clock> startTime;
            double epochTime;
            const unsigned int minimumJointStateMessages; // if less than this many messages have been received, no velocity approximation is run
            unsigned int receivedJointStateMessages;
        };

    } // namespace VelocityApproximation

} // namespace Zyklio

#endif //TRUPHYSICS_TRUVELOCITYAPPROXIMATION_H
