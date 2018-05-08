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
#include <ZyVelocityApproximator.h>

#include <sofa/simulation/Node.h>
#include <sofa/core/objectmodel/BaseContext.h>
#include <sofa/core/objectmodel/ClassInfo.h>
#include <sofa/core/ObjectFactory.h>

#include <SofaMeshCollision/TriangleModel.h>
#include <SofaMeshCollision/LineModel.h>
#include <SofaMeshCollision/PointModel.h>

#include <SofaConstraint/FrictionContact.h>
#ifdef TRUPHYSICS_DEMO
#include <sofa/simulation/common/Simulation.h>
#endif //TRUPHYSICS_DEMO

#include <boost/date_time/posix_time/posix_time.hpp>

//#define TP_VELOCITY_APPROXIMATION_DEBUG

namespace Zyklio
{
    namespace VelocityApproximation
    {

        using namespace sofa;
        using namespace sofa::component;
        using namespace sofa::component::controller;

        using namespace sofa::core::objectmodel;
        using namespace sofa;

        SOFA_DECL_CLASS(TruVelocityApproximator);

        int TruVelocityApproximatorClass = core::RegisterObject("Description Test test.")
        .add< TruVelocityApproximator >()
        //.addAlias("TruVelocityApproximator")
        ;

        TruVelocityApproximator::TruVelocityApproximator()
            : objectState(NULL)
            , collisionStateVec(NULL)
            , objectNodePath(initData(&objectNodePath, "objectNodePath", "Zyklio: Path to the object.")) // TODO: replace this with the proper SOFA way to do this
            , collisionNodePath(initData(&collisionNodePath, "collisionNodePath", "Zyklio: Path to the collision object of the object.")) // TODO: replace this with the proper SOFA way to do this
            , jointNameInJointStatesMsg(initData(&jointNameInJointStatesMsg, "jointNameInJointStatesMsg", "Name of a joint that appears in the joint_states message that is used in velocity approximation. Leave empty for no check."))
            , rosControl(initData(&rosControl, false, "rosControl", "Set to true if control by ROS should be taken into account."))
            , selectedVertices(initData(&selectedVertices, helper::vector< unsigned int >(1, 0u), "selectedVertex", "Select vertex for velocity approximation."))
            , currentVelocityApproximationData(initData(&currentVelocityApproximationData, sofa::helper::vector< currentVelocityApproximationDataType >(), "currentVelocityApproximationData", "Velocity data of the selected vertices."))
            , currentPosition()
            , oldPosVec3Vec()
            , positionChange(false)
            , arbitraryController(NULL)
            , lastResetTime(0.0)
            , rosJointMsgs()
            , oldRosStamp(0u)
            , oldRosTime(0.0)
            , oldIt(rosJointMsgs.begin())
            , rosTimeSteps()
            , lastPushedMsgTime(0.0)
            , lastPushedStamp(0u)
            , indexTornado()
            , testRunIndexTornado()
            , vertexTornadoVec_x()
            , vertexTornadoVec_y()
            , vertexTornadoVec_z()
            , tornadoObjectNameVec()
            , calledByTornado()
            , runVelocityApproximation()
            , selectedVertexInfo(initData(&selectedVertexInfo, "selectedVertexInfo", "Information about the vertex that was selected for velocity approximation."))
            , meshNameVec()
            , epochTime(0.0)
            , minimumJointStateMessages(1u)
            , receivedJointStateMessages(0u)
        {
            rosJointMsgs.resize(80);
            rosTimeSteps.clear();
            rosTimeSteps.resize(ROSTIMESTEPS_USED_FOR_AVERAGE);
                
            // time in seconds since begin of epoch, using chrono::steady_clock
            epochTime = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
#ifdef TP_VELOCITY_APPROXIMATION_DEBUG
            std::cout << std::setprecision(30) << "TruVelocityApproximator epoch time: " << epochTime << std::endl;
#endif
        }

        void TruVelocityApproximator::init()
        {
#ifdef TRUPHYSICS_DEMO
            std::string sceneHash = sofa::simulation::getSimulation()->getSceneHash();

            sofa::helper::hashCheckHelper hch;
            if (!(hch.checkHashCorrectness(sceneHash)))
            {
                std::cout << "This plugin can only be used with the Zyklio demo and the correct demo scene." << std::endl;
                exit(1);
            }
#endif //TRUPHYSICS_DEMO
            Inherit::init();
        }

        void TruVelocityApproximator::bwdInit()
        {                
            // connection to the start via Tornado
            char* tornadoEnv = getenv("TRUSIMRUN_TORNADO");
            char* indexEnv = getenv("TRUSIMRUN_INDEX");
            char* testRunIndexEnv = getenv("TRUSIMRUN_TESTRUNINDEX");
            char* vertexEnv_x = getenv("TRUSIMRUN_VERTEX_X");
            char* vertexEnv_y = getenv("TRUSIMRUN_VERTEX_Y");
            char* vertexEnv_z = getenv("TRUSIMRUN_VERTEX_Z");
            char* objectEnv = getenv("TRUSIMRUN_OBJECT");

            runVelocityApproximation = true; // this is set to false if anything is wrong with the paramters of the velocity approximation

            calledByTornado = (tornadoEnv && (std::string(tornadoEnv).compare("yes") == 0));
            if (calledByTornado)
            {
                // In the following, the parameters of a remote call are processed and checked for correctness.
                // If there is anything wrong, the variable runVelocityApproximation is set to false and
                // the velocity approximation is not run.

                std::cout << "(TruVelocityApproximator::bwdInit) Getting parameters from Tornado." << std::endl;

                if (vertexEnv_x)
                {
                    vertexTornadoVec_x = generateVertexVector(vertexEnv_x);
                    std::cout << "(TruVelocityApproximator::bwdInit) vertex_x: ";
                    for (unsigned int tmp = 0; tmp < vertexTornadoVec_x.size(); tmp++)
                    {
                        std::cout << vertexTornadoVec_x.at(tmp) << " ";
                    }
                    std::cout << std::endl;
                }
                else
                {
                    std::cout << "(TruVelocityApproximator::bwdInit) ERROR: There was a remote call, but no value for the vertex x-coordinate was given. Can't run velocity approximation." << std::endl;
                    runVelocityApproximation = false;
                }

                if (vertexEnv_y)
                {
                    vertexTornadoVec_y = generateVertexVector(vertexEnv_y);
                    std::cout << "(TruVelocityApproximator::bwdInit) vertex_y: ";
                    for (unsigned int tmp = 0; tmp < vertexTornadoVec_y.size(); tmp++)
                    {
                        std::cout << vertexTornadoVec_y.at(tmp) << " ";
                    }
                    std::cout << std::endl;
                }
                else
                {
                    std::cout << "(TruVelocityApproximator::bwdInit) ERROR: There was a remote call, but no value for the vertex y-coordinate was given. Can't run velocity approximation." << std::endl;
                    runVelocityApproximation = false;
                }

                if (vertexEnv_z)
                {
                    vertexTornadoVec_z = generateVertexVector(vertexEnv_z);
                    std::cout << "(TruVelocityApproximator::bwdInit) vertex_z: ";
                    for (unsigned int tmp = 0; tmp < vertexTornadoVec_z.size(); tmp++)
                    {
                        std::cout << vertexTornadoVec_z.at(tmp) << " ";
                    }
                    std::cout << std::endl;
                }
                else
                {
                    std::cout << "(TruVelocityApproximator::bwdInit) ERROR: There was a remote call, but no value for the vertex z-coordinate was given. Can't run velocity approximation." << std::endl;
                    runVelocityApproximation = false;
                }
                        
                if (objectEnv)
                {
                    tornadoObjectNameVec = generateObjectNameVector(objectEnv);
                    std::cout << "(TruVelocityApproximator::bwdInit) object: ";
                    for (unsigned int tmp = 0; tmp < tornadoObjectNameVec.size(); tmp++)
                    {
                        std::cout << tornadoObjectNameVec.at(tmp) << " ";
                    }
                    std::cout << std::endl;
                }
                else
                {
                    std::cout << "(TruVelocityApproximator::bwdInit) ERROR: There was a remote call, but no object name was given. Can't run velocity approximation." << std::endl;
                    runVelocityApproximation = false;
                }

                if (indexEnv)
                {
                    try
                    {
                        indexTornado = std::stoi(indexEnv);
                    }
                    catch (std::invalid_argument)
                    {
                        std::cout << "(TruVelocityApproximator::bwdInit) ERROR: Could not convert input " << indexEnv << " to an integer. Can't run velocity approximation." << std::endl;
                        runVelocityApproximation = false;
                    }
                    catch (std::out_of_range)
                    {
                        std::cout << "(TruVelocityApproximator::bwdInit) ERROR: Input " << indexEnv << " too large. Can't run velocity approximation." << std::endl;
                        runVelocityApproximation = false;
                    }
                    std::cout << "(TruVelocityApproximator::bwdInit) index: " << indexTornado << std::endl;
                }
                else
                {
                    std::cout << "(TruVelocityApproximator::bwdInit) ERROR: There was a remote call, but no point index was given. Can't run velocity approximation." << std::endl;
                    runVelocityApproximation = false;
                }

                if (testRunIndexEnv)
                {
                    try
                    {
                        testRunIndexTornado = std::stoi(testRunIndexEnv);
                    }
                    catch (std::invalid_argument)
                    {
                        std::cout << "(TruVelocityApproximator::bwdInit) ERROR: Could not convert input " << testRunIndexEnv << " to an integer. Can't run velocity approximation." << std::endl;
                        runVelocityApproximation = false;
                    }
                    catch (std::out_of_range)
                    {
                        std::cout << "(TruVelocityApproximator::bwdInit) ERROR: Input " << testRunIndexEnv << " too large. Can't run velocity approximation." << std::endl;
                        runVelocityApproximation = false;
                    }
                    std::cout << "(TruVelocityApproximator::bwdInit) testRunIndex: " << testRunIndexTornado << std::endl;
                }
                else
                {
                    std::cout << "(TruVelocityApproximator::bwdInit) ERROR: There was a remote call, but no testrun index was given. Can't run velocity approximation." << std::endl;
                    runVelocityApproximation = false;
                }

                if ( !( (vertexTornadoVec_x.size() == vertexTornadoVec_y.size()) && (vertexTornadoVec_z.size() == tornadoObjectNameVec.size()) && (vertexTornadoVec_x.size() == tornadoObjectNameVec.size()) ) )
                {
                    std::cout << "(TruVelocityApproximator::bwdInit) ERROR: The vectors for the vertex coordinates and the object names had not all the same size (";
                    std::cout << "vertexTornadoVec_x.size(): " << vertexTornadoVec_x.size();
                    std::cout << ", vertexTornadoVec_y.size(): " << vertexTornadoVec_y.size();
                    std::cout << ", vertexTornadoVec_z.size(): " << vertexTornadoVec_z.size();
                    std::cout << ", tornadoObjectNameVec.size(): " << tornadoObjectNameVec.size();
                    std::cout << "). Can't run velocity approximation." << std::endl;
                    runVelocityApproximation = false;
                }
            }

            // <!-- TODO: replace this with the proper SOFA way to do this 
            std::cout << "(TruVelocityApproximator::bwdInit) objectNodePath: " << objectNodePath.getValue() << std::endl;
            std::cout << "(TruVelocityApproximator::bwdInit) (if it crashes now, this is probably wrong)" << std::endl;
            {
                std::istringstream readPathStream(objectNodePath.getValue());

                simulation::Node* objectNode = dynamic_cast<simulation::Node*>(getContext()->getRootContext());

                while (!readPathStream.eof())
                {
                    std::string tmp;
                    std::getline(readPathStream, tmp, '/');

                    objectNode = objectNode->getChild(tmp); // TODO: find a way to do this, so that SOFA doesn't crash when the path is wrong
                }

                if (objectNode)
                {
                    objectState = dynamic_cast< sofa::component::container::MechanicalObject<defaulttype::RigidTypes>* >(objectNode->getMechanicalState());
                    if (objectState)
                    {
                        currentPosition = objectState->x.getValue();
                    }
                }
            }

            if (calledByTornado && runVelocityApproximation)
            {
                collisionStateVec.clear();

                // The input is the name of a node generated by the Collada loader. This node does not contain the needed mechanical state.
                // What is needed instead is a mechanical state in a sub-node of a sub-node of that node, which has the collision models for this rigid.
                for (unsigned int gna = 0; gna < tornadoObjectNameVec.size(); gna++)
                {
                    std::cout << "(TruVelocityApproximator::bwdInit) Trying to find the mechanical state that corresponds to the object name " << tornadoObjectNameVec.at(gna) << std::endl;

                    simulation::Node* objNode = dynamic_cast<simulation::Node*>(getContext()->getRootContext())->getTreeNode(tornadoObjectNameVec.at(gna));
                    
                    if (objNode)
                    {
                        // look for a Vec3-MechanicalState
                        std::vector<core::behavior::MechanicalState<Vec3Types>*> mechanicalStates;
                        sofa::core::objectmodel::BaseContext::GetObjectsCallBackT<core::behavior::MechanicalState<Vec3Types>, std::vector<core::behavior::MechanicalState<Vec3Types>* > > mechanicalStates_vec(&mechanicalStates);
                        objNode->getContext()->getObjects(TClassInfo<core::behavior::MechanicalState<Vec3Types>>::get(), mechanicalStates_vec, TagSet(), sofa::core::objectmodel::BaseContext::SearchDown);

                        if (mechanicalStates.size() == 0) 
                        {
                            std::cout << "(TruVelocityApproximator::bwdInit) ERROR: No MechanicalState<Vec3Types> instances found! Can't run velocity approximation." << std::endl;
                            runVelocityApproximation = false;
                        }
                        else
                        {
                            std::cout << "(TruVelocityApproximator::bwdInit) MechanicalState<Vec3Types> instances found: " << mechanicalStates.size() << std::endl;

                            // The MechanicalState we need should be the only one with Vec3 types, but to make sure, check if there are any collision models there
                            core::behavior::MechanicalState<Vec3Types>* tmpState = mechanicalStates.at(0);

                            core::CollisionModel::SPtr collModels;
                            tmpState->getContext()->get(collModels, sofa::core::objectmodel::BaseContext::Local);

                            if (collModels)
                            {
                                sofa::component::container::MechanicalObject<defaulttype::Vec3Types>* tmpPnt = dynamic_cast<sofa::component::container::MechanicalObject<defaulttype::Vec3Types>*>(tmpState);
                                if (tmpPnt)
                                {
                                    collisionStateVec.push_back(tmpPnt);
                                }
                                else
                                {
                                    std::cout << "(TruVelocityApproximator::bwdInit) ERROR: Could not find the a Vec3 mechanical state! Can't run velocity approximation." << std::endl;
                                    runVelocityApproximation = false;
                                }
                            }
                            else
                            {
                                std::cout << "(TruVelocityApproximator::bwdInit) ERROR: Could not find the right mechanical state! Can't run velocity approximation." << std::endl;
                                runVelocityApproximation = false;
                            }
                        }
                    }
                    else
                    {
                        std::cout << "(TruVelocityApproximator::bwdInit) ERROR: Could not find a mechanical state that corresponds to " << tornadoObjectNameVec.at(gna) << ". Can't run velocity approximation." << std::endl;
                        runVelocityApproximation = false;
                    }
                }
            }
            else
            {
                collisionStateVec.resize(1);

                std::cout << "(TruVelocityApproximator::bwdInit) collisionNodePath: " << collisionNodePath.getValue() << std::endl;
                std::cout << "(TruVelocityApproximator::bwdInit) (if it crashes now, this is probably wrong)" << std::endl;

                std::istringstream readPathStream(collisionNodePath.getValue());
                simulation::Node* objectNode = NULL;
                objectNode = dynamic_cast<simulation::Node*>(getContext()->getRootContext());

                while (!readPathStream.eof())
                {
                    std::string tmp;
                    std::getline(readPathStream, tmp, '/');

                    objectNode = objectNode->getChild(tmp); // TODO: find a way to do this, so that SOFA doesn't crash when the path is wrong
                }

                if (objectNode)
                {
                    collisionStateVec.at(0) = dynamic_cast< sofa::component::container::MechanicalObject<defaulttype::Vec3Types>* >(objectNode->getMechanicalState());
                }
            }

            oldPosVec3Vec.resize(collisionStateVec.size());
            for (unsigned int arg = 0; arg < collisionStateVec.size(); arg++)
            {
                if (collisionStateVec.at(arg))
                {
                    oldPosVec3Vec.at(arg) = collisionStateVec.at(arg)->x.getValue();
                }
            }
                
            // -->

            if (objectState)
            {
                std::cout << "(TruVelocityApproximator::bwdInit) Found mechanical object: " << objectState->getName() << std::endl;
            }
            else
            {
                std::cout << "(TruVelocityApproximator::bwdInit) Mechanical object with path \"" << objectNodePath.getValue() << "\" not found. (Probably not alarming)" << std::endl;
            }

            identificationString = boost::posix_time::to_simple_string(boost::posix_time::microsec_clock::local_time()); // is not set anywhere else
            if (calledByTornado) // if called by Tornado replace the selected vertex list from the scn file with the one given by the Tornado call
            {
                helper::vector< unsigned int > * tmp = selectedVertices.beginEdit();
                tmp->resize(collisionStateVec.size());
                selectedVertices.endEdit();
            }

            for (unsigned int mah = 0; mah < collisionStateVec.size(); mah++)
            {
                if (collisionStateVec.at(mah))
                {
                    if (calledByTornado)
                    {
                        std::cout << "(TruVelocityApproximator::bwdInit) Found mechanical collision object corresponding to the name " << tornadoObjectNameVec.at(mah) << ": " << collisionStateVec.at(mah)->getName() << std::endl;
                        std::stringstream tmpStr;
                        tmpStr << collisionStateVec.at(mah)->getName() << " (" << tornadoObjectNameVec.at(mah) << ")";
                        meshNameVec.push_back(tmpStr.str());
                    }
                    else
                    {
                        std::cout << "(TruVelocityApproximator::bwdInit) Found mechanical collision object corresponding to the path " << collisionNodePath.getValue() << ": " << collisionStateVec.at(mah)->getName() << std::endl;
                        std::stringstream tmpStr;
                        tmpStr << collisionStateVec.at(mah)->getName();
                        meshNameVec.push_back(tmpStr.str());
                    }

                    if (runVelocityApproximation)
                    {

                        if (calledByTornado) // if called by Tornado replace the selected vertex list from the scn file with the one given by the Tornado call
                        {
                            Vector3 selectedVertex(vertexTornadoVec_x.at(mah), vertexTornadoVec_y.at(mah), vertexTornadoVec_z.at(mah));

#ifdef TP_VELOCITY_APPROXIMATION_DEBUG
                            std::cout << "selectedVertex" << ": " << selectedVertex << std::endl;
#endif

                            double minDist = DBL_MAX;
                            unsigned int minDist_index = collisionStateVec.at(mah)->x.getValue().size() + 1;
                            for (unsigned int posi = 0; posi < collisionStateVec.at(mah)->x.getValue().size(); posi++)
                            {
                                double currentDist = (selectedVertex - Vector3(collisionStateVec.at(mah)->x.getValue().at(posi))).norm2();
                                if (currentDist < minDist)
                                {
                                    minDist = currentDist;
                                    minDist_index = posi;
                                }
                            }

                            helper::vector< unsigned int > * tmp = selectedVertices.beginEdit();
                            tmp->at(mah) = minDist_index;
                            selectedVertices.endEdit();

                            std::stringstream tmpStr;
                            tmpStr << collisionStateVec.at(mah)->x.getValue().at(minDist_index) << " (index " << minDist_index << ")";
                            std::cout << "(TruVelocityApproximator::bwdInit) Due to the given position (" << selectedVertex << "), the following vertex on the sofa-mesh was selected: " << tmpStr.str() << std::endl;
                            sofa::helper::vector < std::string >* shvssPnt = selectedVertexInfo.beginEdit();
                            shvssPnt->push_back(tmpStr.str());
                            selectedVertexInfo.endEdit();
                        }

                        sofa::helper::vector< currentVelocityApproximationDataType >* velApproxData = currentVelocityApproximationData.beginEdit();
                        velApproxData->clear();

                        {
                            unsigned int currentSelectedVertex;

                            currentSelectedVertex = selectedVertices.getValue().at(mah);

                            if (currentSelectedVertex >= collisionStateVec.at(mah)->x.getValue().size())
                            {
                                serr << "(TruVelocityApproximator::bwdInit) WARNING: Selected vertex " << currentSelectedVertex << " is bigger than the number of vertices in the mesh (" << collisionStateVec.at(mah)->x.getValue().size() << "). Setting this selected vertex to 0." << std::endl;
                                if (selectedVertices.getValue().size() > 1)
                                {
                                    currentSelectedVertex = 0;
                                }
                            }

                            std::pair<identificationData, velocityData> velDataWIdent;
                            identificationData* velDataIdentificationVec = &(velDataWIdent.first);
                            velDataIdentificationVec->at(0) = identificationString; 
                            velDataIdentificationVec->at(1) = meshNameVec.at(mah);
                            velDataIdentificationVec->at(2) = std::to_string(currentSelectedVertex);
                            if (calledByTornado)
                            {
                                velDataIdentificationVec->at(3) = std::to_string(indexTornado);
                                velDataIdentificationVec->at(4) = std::to_string(testRunIndexTornado);
                            }
                            else
                            {
                                // these two values are only provided by a remote call
                                velDataIdentificationVec->at(3) = "0";
                                velDataIdentificationVec->at(4) = "0";
                            }

                            // initial values for the velocity data vector
                            defaulttype::Vec3Types::Coord currentPoint = collisionStateVec.at(mah)->x.getValue().at(currentSelectedVertex);

                            velocityData* velData = &(velDataWIdent.second);
                            velData->at(0) = 0.0;
                            velData->at(1) = 0.0;
                            velData->at(2) = 0.0;
                            velData->at(3) = 0.0;
                            velData->at(4) = 0.0;
                            velData->at(5) = currentPoint[0];
                            velData->at(6) = currentPoint[1];
                            velData->at(7) = currentPoint[2];

                            velApproxData->push_back(createOutputArray(velDataWIdent));
                        }
                        currentVelocityApproximationData.endEdit();

//#ifdef TP_VELOCITY_APPROXIMATION_DEBUG
//                            std::cout << "Initial positions of tracked objects:" << std::endl;
//                            for (unsigned int gnuh = 0; gnuh < collisionStateVec.at(mah)->getPosition().size(); gnuh++)
//                            {
//                                std::cout << collisionStateVec.at(mah)->getName() << ": " << collisionStateVec.at(mah)->getPosition().at(gnuh) << std::endl;
//                            }
//#endif
                            
                    }
                }
                else
                {
                    if (calledByTornado)
                    {
                        std::cout << "(TruVelocityApproximator::bwdInit) Mechanical object with name \"" << tornadoObjectNameVec.at(mah) << "\" not found." << std::endl;
                    }
                    else
                    {
                        std::cout << "(TruVelocityApproximator::bwdInit) Mechanical object with path \"" << collisionNodePath.getValue() << "\" not found." << std::endl;
                    }
                }
            }

            std::vector<ArbitraryController*> arbitraryControllers;
            sofa::core::objectmodel::BaseContext::GetObjectsCallBackT<ArbitraryController, std::vector<ArbitraryController* > > arbitraryControllers_vec(&arbitraryControllers);
            getContext()->getObjects(TClassInfo<ArbitraryController>::get(), arbitraryControllers_vec, TagSet(), sofa::core::objectmodel::BaseContext::SearchRoot);

            std::cout << "(TruVelocityApproximator::bwdInit) ArbitraryController instances found: " << arbitraryControllers.size() << std::endl;

            if (arbitraryControllers.size() == 0) {
                serr << "(TruVelocityApproximator::bwdInit) No Arbitrary Contoller found!" << std::endl;
                runVelocityApproximation = false;
            }
            else
            {
                arbitraryController = arbitraryControllers.at(0);
            }

            if (runVelocityApproximation)
            {
                std::cout << "(TruVelocityApproximator::bwdInit) All paramaters for velocity approximation are ok." << std::endl;
            }
            else
            {
                std::cout << "(TruVelocityApproximator::bwdInit) Not all paramaters for velocity approximation are ok. Can't run velocity approximation." << std::endl;
            }

            rosTimeSteps.clear();
        }

        void TruVelocityApproximator::reinit()
        {
            // TODO: This is probably incomplete
            currentPosition.clear();
            positionChange = false;

            for (unsigned int wy = 0; wy < oldPosVec3Vec.size(); wy++)
            {
                oldPosVec3Vec.at(wy).clear();
            }

            bwdInit();
        }

        // TODO: This function is old code that was used to move an object to a specific position (with the old ROSConnector, to set an object position via coordinates given in a ros topic). If this functionality is needed in the future, move this code into it's own class.
        //void TruVelocityApproximator::moveObjectToPosition(simulation::common::MechanicalOperations& mop, sofa::core::behavior::MultiVecCoord& pos/*, sofa::core::behavior::MultiVecCoord& freePos*/)
        //{
        //    if (doWhat.getValue().compare("movement") == 0)
        //    {
        //        if (objectState && positionChange)
        //        {
        //            mop.propagateX(pos, true);

        //            std::cout << "Setting position of " << objectState->getName() << " to " << currentPosition << std::endl;
        //            objectState->setPosition(currentPosition);
        //            positionChange = false;

        //            mop.propagateX(pos, true);
        //        }
        //    }

        //    return;
        //}

        void TruVelocityApproximator::approximateVelocity(simulation::common::MechanicalOperations& mop, sofa::core::behavior::MultiVecCoord& pos)
        {            
            for (unsigned int urg = 0; urg < collisionStateVec.size(); urg++)
            {
                if (/*!objectState &&*/ !collisionStateVec.at(urg))
                {
                    return;
                }
            }

                

            if (!runVelocityApproximation)
            {
                return;
            }
                

            if (rosControl.getValue() && (receivedJointStateMessages < minimumJointStateMessages))
            {
                return;
            }
                

            // for rosJointMsgs explanation see header
            currentRosStamp = rosJointMsgs.back().first.first;
            if (rosControl.getValue() && (currentRosStamp == oldRosStamp)) // TODO: there might be something wrong here (see where oldRosStamp is used)
            {
                std::cout << "No update of velocity approximation since last known ROS time stamp (" << oldRosStamp << ") has not changed." << std::endl;
                        
                sofa::helper::vector< currentVelocityApproximationDataType >* curVelAprDat = currentVelocityApproximationData.beginEdit();
                curVelAprDat->clear();
                currentVelocityApproximationData.endEdit();
            }
            else
            {
                {
                    std::cout << "============\nVelocity approximation for: ";
                    for (unsigned int irg = 0; irg < collisionStateVec.size(); irg++)
                    {
                        if (collisionStateVec.at(irg))
                        {
                            std::cout << collisionStateVec.at(irg)->getName() << ", " << std::endl;
                        }
                    }

                    if (rosControl.getValue())
                    {
                        // look for last known joint_states message
#ifdef TP_VELOCITY_APPROXIMATION_DEBUG
                        std::cout << "\ncurrent joint_states message time stamp: " << currentRosStamp << std::endl;
                        std::cout << "looking for last known joint_states message (time stamp: " << oldRosStamp << ")" << std::endl;
#endif
                        missedRosJointMsgs.clear();
                        bool seqFound = false;

                        // copy unprocessed joint_states messages into a separate data structure
                        boost::circular_buffer<jointMsg>::iterator it = (rosJointMsgs.end() - 1);
                        for (; it != rosJointMsgs.begin() && !seqFound; it--)
                        {
                            missedRosJointMsgs.push_back((*it));

                            seqFound = ((*it).first.first == oldRosStamp);
                        }

                        double tmpStamp = oldRosStamp; // TODO: make this better (just put oldRosStamp instead of tmpStamp?)
                        //oldRosStamp = missedRosJointMsgs.back().first.first; // TODO: there might be something wrong here (see where oldRosStamp is used) (might have fixed this by commenting out this line)

                        if (!seqFound)
                        {
                            std::cout << "WARNING (TruVelocityApproximator::approximateVelocity): Last known joint_states message time stamp not found in known joint_states messages. Using all known messages." << std::endl;
                        }
                        else
                        {
                            missedRosJointMsgs.push_back(*it); // start with the last processed message
                        }

                        sofa::helper::vector< currentVelocityApproximationDataType >* velApproxData = currentVelocityApproximationData.beginEdit();
                        velApproxData->clear();

                        // Instead of using the time difference between the current joint_states message und the previous one,
                        // we use the average of time differences between the last ROSTIMESTEPS_USED_FOR_AVERAGE messages.
                        // The reason for that is that the timestamps in joint_states messages seem to be not always equal to
                        // the real time on which the content of the message was generated. 
                        // This solution is not perfect but gives adequate results.
                        // TODO: Find a better way to deal with this
                        double averageTimeStep = 0.0;
                        if (rosTimeSteps.size() <= 1)
                        {
                            return; // no velocity approximation without a time step
                        }
                        for (unsigned int u = 1; u < rosTimeSteps.size(); u++) // ignoring the first one, because it's the fastest way to deal with the problem that the very first entry in the list (when the simulation starts) is nonsense
                        {
                            averageTimeStep += rosTimeSteps.at(u);
                        }

                        averageTimeStep = averageTimeStep / (rosTimeSteps.size() - 1);

#ifdef TP_VELOCITY_APPROXIMATION_DEBUG
                        std::cout << std::setprecision(30) << "Average ROS time step: " << averageTimeStep << std::endl;
#endif


#ifdef TP_VELOCITY_APPROXIMATION_DEBUG
                        std::cout << "\nReplaying missed joint_states messages:" << std::endl;
#endif
                        //for (int c = std::max(missedRosJointMsgs.size() - 3, 0ull); c >= 0; c--)
                        for (int c = std::max<int>(missedRosJointMsgs.size() - 3, 0); c >= 0; c--)
                        {
                                
#ifdef TP_VELOCITY_APPROXIMATION_DEBUG
                            std::cout << "Sequence number: " << missedRosJointMsgs.at(c).first.first << " " /*<< std::endl*/;
#endif
                            sofa::helper::vector<jointData> tmpJntDatVec = missedRosJointMsgs.at(c).second;

                            double stampDiff;
                            if (c == (missedRosJointMsgs.size() - 3))
                            {
                                stampDiff = missedRosJointMsgs.at(c).first.first - tmpStamp;
                            }
                            else
                            {
                                stampDiff = missedRosJointMsgs.at(c).first.first - missedRosJointMsgs.at(c + 1).first.first;
                            }
                            double currentTimeStep = averageTimeStep * stampDiff;

#ifdef TP_VELOCITY_APPROXIMATION_DEBUG
                            std::cout << "stampDiff:" << stampDiff << ", missedRosJointMsgs.at(c).first.first:" << missedRosJointMsgs.at(c).first.first << ", tmpStamp:" << tmpStamp << std::endl;
#endif

                            if (arbitraryController != NULL) // just to be safe - arbitraryController can't really be a null pointer here
                            {
                                for (unsigned int g = 0; g < tmpJntDatVec.size(); g++)
                                {
                                    arbitraryController->setRotValueRadByName(tmpJntDatVec.at(g).first, tmpJntDatVec.at(g).second);
                                }

                                arbitraryController->applyController();
                                mop.propagateX(pos);
                            }

                            double currentTime = adjustTime(missedRosJointMsgs.at(c).first.second);
#ifdef TP_VELOCITY_APPROXIMATION_DEBUG
                            std::cout << "currentTime: " << currentTime << ", currentTimeStep: " << currentTimeStep << std::endl;
#endif

                            for (unsigned int noo = 0; noo < collisionStateVec.size(); noo++)
                            {
                                ////////////std::cout << "loop info:\n     start: " << 0 << "\n   current: " << noo << "\n      stop: " << collisionStateVec.size() << std::endl;
                                defaulttype::Vec3Types::VecCoord curPosVec3 = collisionStateVec.at(noo)->x.getValue();
                                {
#ifdef TP_VELOCITY_APPROXIMATION_DEBUG
                                    std::cout << "currentState: " << collisionStateVec.at(noo)->getName() << std::endl;
#endif
                                    unsigned int currentSelectedVertex = selectedVertices.getValue().at(noo);  

                                    std::pair<identificationData, velocityData> velDataWIdent;
                                    identificationData* velDataIdentificationVec = &(velDataWIdent.first);
                                    velDataIdentificationVec->at(0) = identificationString;
                                    velDataIdentificationVec->at(1) = meshNameVec.at(noo);
                                    velDataIdentificationVec->at(2) = std::to_string(currentSelectedVertex);
                                    if (calledByTornado)
                                    {
                                        velDataIdentificationVec->at(3) = std::to_string(indexTornado);
                                        velDataIdentificationVec->at(4) = std::to_string(testRunIndexTornado);
                                    }
                                    else
                                    {
                                        // these two values are only provided by a remote call
                                        velDataIdentificationVec->at(3) = "0"; 
                                        velDataIdentificationVec->at(4) = "0";
                                    }

                                    if (collisionStateVec.at(noo))
                                    {
                                        unsigned int i = currentSelectedVertex;

                                        defaulttype::Vec3Types::Coord currentPoint = curPosVec3.at(i);
                                        defaulttype::Vec3Types::Coord oldPoint = oldPosVec3Vec.at(noo).at(i);

                                        defaulttype::Vec3Types::Deriv approxVelocity;

                                        for (unsigned int j = 0; j < approxVelocity.size(); j++)
                                        {
                                            approxVelocity[j] = (currentPoint[j] - oldPoint[j]) / currentTimeStep;
                                        }

                                        /*approxVelocity = currentPoint - oldPoint;
                                        approxVelocity[0] = approxVelocity[0] / currentTimeStep;
                                        approxVelocity[1] = approxVelocity[1] / currentTimeStep;
                                        approxVelocity[2] = approxVelocity[2] / currentTimeStep;*/

                                        velocityData* velData = &(velDataWIdent.second);
                                        velData->at(0) = currentTime;
                                        velData->at(1) = approxVelocity[0];
                                        velData->at(2) = approxVelocity[1];
                                        velData->at(3) = approxVelocity[2];
                                        velData->at(4) = approxVelocity.norm();
                                        velData->at(5) = currentPoint[0];
                                        velData->at(6) = currentPoint[1];
                                        velData->at(7) = currentPoint[2];
                                    }

                                    velApproxData->push_back(createOutputArray(velDataWIdent));
                                }

                                oldPosVec3Vec.at(noo) = curPosVec3;
                            }
                            // hm //oldTime = currentTime;
                            oldRosStamp = currentRosStamp; // TODO: there might be something wrong here (see where oldRosStamp is used)
                                
                        }
                        currentVelocityApproximationData.endEdit();
                    }
                    else
                    {
                        double timeStep = getContext()->getDt();

                        for (unsigned int naa = 0; naa < collisionStateVec.size(); naa++)
                        {
                            defaulttype::Vec3Types::VecCoord curPosVec3 = collisionStateVec.at(naa)->x.getValue();

                            sofa::helper::vector< currentVelocityApproximationDataType >* velApproxData = currentVelocityApproximationData.beginEdit();
                            velApproxData->clear();
                            for (unsigned int selVert = 0; selVert < selectedVertices.getValue().size(); selVert++)
                            {
                                unsigned int currentSelectedVertex = selectedVertices.getValue().at(selVert);

                                std::pair<identificationData, velocityData> velDataWIdent;
                                identificationData* velDataIdentificationVec = &(velDataWIdent.first);
                                velDataIdentificationVec->at(0) = identificationString;
                                velDataIdentificationVec->at(1) = meshNameVec.at(naa);
                                velDataIdentificationVec->at(2) = std::to_string(currentSelectedVertex);
                                if (calledByTornado)
                                {
                                    velDataIdentificationVec->at(3) = std::to_string(indexTornado);
                                    velDataIdentificationVec->at(4) = std::to_string(testRunIndexTornado);
                                }
                                else
                                {
                                    velDataIdentificationVec->at(3) = "0";
                                    velDataIdentificationVec->at(4) = "0";
                                }

                                defaulttype::Vec3Types::Coord currentPoint = curPosVec3.at(currentSelectedVertex);
                                defaulttype::Vec3Types::Coord oldPoint = oldPosVec3Vec.at(naa).at(currentSelectedVertex);

                                defaulttype::Vec3Types::Deriv approxVelocity;
                                for (unsigned int j = 0; j < approxVelocity.size(); j++)
                                {
                                    approxVelocity[j] = (currentPoint[j] - oldPoint[j]) / timeStep;
                                }

                                velocityData* velData = &(velDataWIdent.second);
                                velData->at(0) = getContext()->getTime();
                                velData->at(1) = approxVelocity[0];
                                velData->at(2) = approxVelocity[1];
                                velData->at(3) = approxVelocity[2];
                                velData->at(4) = approxVelocity.norm();
                                velData->at(5) = currentPoint[0];
                                velData->at(6) = currentPoint[1];
                                velData->at(7) = currentPoint[2];

                                velApproxData->push_back(createOutputArray(velDataWIdent));
                            }
                            oldPosVec3Vec.at(naa) = curPosVec3;
                            currentVelocityApproximationData.endEdit();
                        }
                    }
                        
                }
            }
            
            std::cout << "Velocity approximation finished." << std::endl;
        }


        void TruVelocityApproximator::pushJointMsg(jointMsg theMsg)
        {
            receivedJointStateMessages++;

#ifdef TP_VELOCITY_APPROXIMATION_DEBUG
            std::cout << std::setprecision(30) << "pushing new message: " << theMsg.first.first << ", " << theMsg.first.second << std::endl;
#endif
            rosJointMsgs.push_back(theMsg);
                
            double adjustedTime = adjustTime(theMsg.first.second);
                
            double timeDiff = adjustedTime - lastPushedMsgTime;
            double stampDiff = theMsg.first.first - lastPushedStamp;

            if ((timeDiff > 0.000001) && (stampDiff > 0))
            {
#ifdef TP_VELOCITY_APPROXIMATION_DEBUG
                std::cout << "adjustedTime: " << adjustedTime << ", timeDiff: " << timeDiff << ", sequence: " << theMsg.first.first << ", stampDiff: " << stampDiff << ", joints: ";
                for (unsigned int qqqqqq = 0; qqqqqq < theMsg.second.size() ; qqqqqq++)
                {
                std::cout << theMsg.second.at(qqqqqq).first << ", ";
                }
                std::cout << std::endl;
#endif
                rosTimeSteps.push_back(timeDiff / stampDiff); // divide by stamp difference to account for messages that are lost (or never sent)
            }
                
            lastPushedMsgTime = adjustedTime;
            lastPushedStamp = theMsg.first.first;
        };

        TruVelocityApproximator::currentVelocityApproximationDataType TruVelocityApproximator::createOutputArray(const std::pair< identificationData, velocityData>& velocityDataPair)
        {
            currentVelocityApproximationDataType outputArray;

            outputArray.at(0) = velocityDataPair.first.at(0);   // id string
            outputArray.at(1) = velocityDataPair.first.at(1);   // mesh name
            outputArray.at(2) = velocityDataPair.first.at(2);   // selected vertex (already a string at this point)
            outputArray.at(3) = velocityDataPair.first.at(3);   // point index (already a string at this point)
            outputArray.at(4) = velocityDataPair.first.at(4);   // testrun index (already a string at this point)
            outputArray.at(5) = std::to_string(velocityDataPair.second.at(0));  // sim time
            outputArray.at(6) = std::to_string(velocityDataPair.second.at(1));  // velocity x
            outputArray.at(7) = std::to_string(velocityDataPair.second.at(2));  // velocity y
            outputArray.at(8) = std::to_string(velocityDataPair.second.at(3));  // velocity z
            outputArray.at(9) = std::to_string(velocityDataPair.second.at(4));  // velocity magnitude
            outputArray.at(10) = std::to_string(velocityDataPair.second.at(5));  // position x
            outputArray.at(11) = std::to_string(velocityDataPair.second.at(6));  // position y
            outputArray.at(12) = std::to_string(velocityDataPair.second.at(7));  // position z

#ifdef TP_VELOCITY_APPROXIMATION_DEBUG
            std::cout << "outputArray: " << outputArray << std::endl;
#endif

            return outputArray;
        }

        // Object names arrive as a comma separated list of unknown length.
        // What we need is an actual vector of strings.
        std::vector<std::string> TruVelocityApproximator::generateObjectNameVector(char* vertexInput)
        {
            std::vector<std::string> vertexStrVec;
            std::string vertexStr(vertexInput);
            int pos = vertexStr.find(',');
            while (pos != std::string::npos)
            {
                if (vertexStr.size() > 0)
                {
                    std::string nextSubStr = vertexStr.substr(0, pos);
                    if (nextSubStr.size() > 0)
                    {
                        vertexStrVec.push_back(nextSubStr);
                    }
                    vertexStr = vertexStr.substr(pos + 1);
                }
                pos = vertexStr.find(',');
            }
            if (vertexStr.size() > 0)
            {
                vertexStrVec.push_back(vertexStr);
            }

            return vertexStrVec;
        }

        // Vertex values arrive as a comma separated list of unknown length.
        // What we need is an actual vector of doubles.
        std::vector<double> TruVelocityApproximator::generateVertexVector(char* vertexInput)
        {
            try
            {
                std::vector<double> vertexDblVec;
                std::string vertexStr(vertexInput);
                int pos = vertexStr.find(',');
                while (pos != std::string::npos)
                {
                    if (vertexStr.size() > 0)
                    {
                        std::string nextSubStr = vertexStr.substr(0, pos);
                        if (nextSubStr.size() > 0)
                        {
                            vertexDblVec.push_back(std::stod(nextSubStr));
                        }
                        vertexStr = vertexStr.substr(pos + 1);
                    }
                    pos = vertexStr.find(',');
                }
                if (vertexStr.size() > 0)
                {
                    vertexDblVec.push_back(std::stod(vertexStr));
                }

                return vertexDblVec;
            }
            catch (std::invalid_argument)
            {
                std::cout << "(TruVelocityApproximator::bwdInit) ERROR: Could not convert input " << vertexInput << " to integers. Can't run velocity approximation." << std::endl;
                runVelocityApproximation = false;
            }
            catch (std::out_of_range)
            {
                std::cout << "(TruVelocityApproximator::bwdInit) ERROR: Input " << vertexInput << " contains an element that is too large. Can't run velocity approximation." << std::endl;
                runVelocityApproximation = false;
            }
        }

        double TruVelocityApproximator::adjustTime(double time)
        {
            if (time > epochTime)
            {
                return time - epochTime;
            }
            else
            {
                return time;
            }
        }

    } // namespace VelocityApproximation

} // namespace Zyklio
