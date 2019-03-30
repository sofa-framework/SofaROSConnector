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

#include "ZyROSSimulationAnalyzer.h"

#include <sofa/core/objectmodel/ClassInfo.h>
#include <sofa/core/ObjectFactory.h>

#include <SofaBaseMechanics/MechanicalObject.h>
#include "sofa/helper/vector_algebra.h"

#ifdef ZYKLIO_DEMO
#include <sofa/simulation/common/Simulation.h>
#endif //ZYKLIO_DEMO

namespace Zyklio
{
    namespace SimulationAnalysis
    {

        using namespace sofa;

        SOFA_DECL_CLASS(ZyROSSimulationAnalyzer);

        int TruSimulationAnalyzerClass = core::RegisterObject("Component to determine if a simulation was successful")
        .add< ZyROSSimulationAnalyzer >()
        .addAlias("TruCorder")
        ;

        ZyROSSimulationAnalyzer::ZyROSSimulationAnalyzer()
            : targetName(initData(&targetName, "targetName", "Name of the object that is checked."))
            , targetMinHeight(initData(&targetMinHeight, -100.0, "targetMinHeight", "Minimum Z-value the checked object needs to reach, for the simulation to be considered successful."))
            , targetMovementTolerance(initData(&targetMovementTolerance, 1.0, "targetMovementTolerance", "How far the object is allowed to move each iteration and still be considered sucessfully gripped."))
            , targetIterations(initData(&targetIterations, 10u, "targetIterations", "How many iterations the gripped object needs to stand still to be considered sucessfully gripped."))
            , linInput(0.0)
            , rotInput(0.0)
        {
            /*targetName = "falling_1_Object";
            targetMinHeight = -100.0;
            targetIterations = 10;
            targetMovementTolerance = 1.0;*/

            // TODO: get these from ROS topic
            /*linInput = 0.2;
            rotInput = 0.12;*/

            oldPos = defaulttype::Vec3d(0, 0, 0);
            targetCounter = 0;

            //theTime = boost::posix_time::microsec_clock::local_time();
            boost::posix_time::ptime theTime = boost::posix_time::microsec_clock::local_time();
            boost::posix_time::ptime epoch(boost::gregorian::date(2017, 1, 1)); // 2017 instead of 1970, so that the numbers don't get so big
            timeAtStart = (theTime - epoch).total_seconds();
        }

        void ZyROSSimulationAnalyzer::init()
        {
#ifdef ZYKLIO_DEMO
            std::string sceneHash = sofa::simulation::getSimulation()->getSceneHash();

            sofa::helper::hashCheckHelper hch;
            if (!(hch.checkHashCorrectness(sceneHash)))
            {
                std::cout << "This plugin can only be used with the Zyklio demo and the correct demo scene." << std::endl;
                exit(1);
            }
#endif //ZYKLIO_DEMO
            Inherit::init();
        }

        void ZyROSSimulationAnalyzer::bwdInit()
        {
            Inherit::bwdInit();

            connectionManagerFound = publishingHandler.setROSConnectionManagerByContext(getContext());

            if (connectionManagerFound)
            {
                simResultPublisher = new Zyklio::ROSConnector::ZyROSFloat32MultiArrayPublisher(publishingHandler.getROSNodeHandle(), "truSimResult");
                publishingHandler.registerPublisher(simResultPublisher);
            }
        }

        void ZyROSSimulationAnalyzer::reinit()
        {
            
        }

        void ZyROSSimulationAnalyzer::analyzeSimulationStateAndPublishResult()
        {        

            simulation::Node* root = dynamic_cast<simulation::Node*>(getContext()->getRootContext());
            if (root == NULL) return;

            std::vector<component::container::MechanicalObject<defaulttype::RigidTypes>*> mechanicalObjects;
            root->getTreeObjects<component::container::MechanicalObject<defaulttype::RigidTypes> >(&mechanicalObjects);

            std_msgs::Float32MultiArray msgFloat32MultiArray;
            std::vector<float> dataVec;
            std::vector<std_msgs::MultiArrayDimension> dim;

            unsigned int numberOfValuesPerRigid = 6;
            unsigned int numberOfRigids = /*mechanicalObjects.size()*/ 1;

            unsigned int dataDim = 0;

            dataVec.resize(numberOfRigids * numberOfValuesPerRigid);

            /*dim.resize(2);
            dim.at(0).label = "target object: " + targetName;
            dim.at(0).size = numberOfRigids;
            dim.at(0).stride = numberOfRigids * numberOfValuesPerRigid;*/

            dim.resize(1);

            dim.at(dataDim).label = "success, linear input, rotational input, object z value (object name is '" + targetName.getValue() + "'), timestamp at sofa init";
            dim.at(dataDim).size = numberOfValuesPerRigid;
            dim.at(dataDim).stride = numberOfValuesPerRigid;

            bool objectFound = false;
            unsigned int rigidCount = 0;
            for (std::vector<component::container::MechanicalObject<defaulttype::RigidTypes>*>::const_iterator it = mechanicalObjects.begin(); it != mechanicalObjects.end(); it++)
            {
                if ((*it)->getName().compare(targetName.getValue()) == 0)
                {
                    std::string rigidName = (*it)->getName();
                    defaulttype::Vec3d objPosition(((*it)->x.getValue())[0][0], ((*it)->x.getValue())[0][1], ((*it)->x.getValue())[0][2]);
                    //defaulttype::Quaternion objOrientation(((*it)->getPosition())[0][3], ((*it)->getPosition())[0][4], ((*it)->getPosition())[0][5], ((*it)->getPosition())[0][6]);

                    float success = 0.0;
                    float currentHeight = objPosition.at(2);

                    if (currentHeight > targetMinHeight.getValue())
                    {
                        if ((objPosition - oldPos).norm2() < targetMovementTolerance.getValue())
                        {
                            targetCounter++;
                        }
                        else
                        {
                            targetCounter = 0;
                        }

                        if (targetCounter > targetIterations.getValue())
                        {
                            success = 1.0;
                        }
                        else
                        {
                            success = 0.0;
                        }
                    }

                    dataVec.at((rigidCount* dim[dataDim].stride) + 0) = success;
                    dataVec.at((rigidCount* dim[dataDim].stride) + 1) = linInput;
                    dataVec.at((rigidCount* dim[dataDim].stride) + 2) = rotInput;
                    dataVec.at((rigidCount* dim[dataDim].stride) + 3) = currentHeight;
                    dataVec.at((rigidCount* dim[dataDim].stride) + 4) = timeAtStart;
                    dataVec.at((rigidCount* dim[dataDim].stride) + 5) = getContext()->getTime();

                    objectFound = true;
                    oldPos = objPosition;
                    break;
                }
            }

            if (!objectFound)
            {
                std::cout << "(ZyklioAnimationLoop::analyzeSimulationStateAndPublishResult) Rigid object '" << targetName << "' not found in scene. Publishing zeroes." << std::endl;
                dataVec.at((rigidCount* dim[dataDim].stride) + 0) = 0.0;
                dataVec.at((rigidCount* dim[dataDim].stride) + 1) = 0.0;
                dataVec.at((rigidCount* dim[dataDim].stride) + 2) = 0.0;
                dataVec.at((rigidCount* dim[dataDim].stride) + 3) = 0.0;
                dataVec.at((rigidCount* dim[dataDim].stride) + 4) = timeAtStart;
                dataVec.at((rigidCount* dim[dataDim].stride) + 5) = getContext()->getTime();
            }

            msgFloat32MultiArray.data = dataVec;
            msgFloat32MultiArray.layout.data_offset = 0;
            msgFloat32MultiArray.layout.dim = dim;

            simResultPublisher->publishMessage(msgFloat32MultiArray);

        }
    } // namespace VelocityApproximation

} // namespace Zyklio
