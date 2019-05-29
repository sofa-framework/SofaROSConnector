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
#ifndef SOFA_COMPONENT_INTERACTIONFORCEFIELD_ZyColladaToolHandler_INL
#define SOFA_COMPONENT_INTERACTIONFORCEFIELD_ZyColladaToolHandler_INL

#include "ZyColladaToolHandler.h"

#include <sofa/simulation/Node.h>
#include <sofa/core/objectmodel/BaseContext.h>
#include <sofa/core/objectmodel/ClassInfo.h>

namespace sofa
{

    namespace component
    {

        namespace controller
        {
            using namespace sofa::core::objectmodel;
            using namespace sofa;

            template<class DataTypes>
            ZyColladaToolHandler<DataTypes>::ZyColladaToolHandler()
                : armatureEndStateV()
                , toolStateV()
                , oldPosV()
                , armatureEndNodePath(initData(&armatureEndNodePath, "armatureEndNodePath", "Zyklio: Path to the end of the kinematics chain. Start with first node under root, separate nodes with slashes, don't put a slash at the end.")) // TODO: replace this with the proper SOFA way to do this
                , toolNodePath(initData(&toolNodePath, "toolNodePath", "Zyklio: Path to the root node of the tool. Start with first node under root, separate nodes with slashes, don't put a slash at the end.")) // TODO: replace this with the proper SOFA way to do this
                , canHandleTools(false)
            {
            }

            template<class DataTypes>
            void ZyColladaToolHandler<DataTypes>::init()
            {
                Inherit::init();
            }

            template<class DataTypes>
            void ZyColladaToolHandler<DataTypes>::bwdInit()
            {

                sofa::helper::vector<std::string> armatureEndNodePathV = armatureEndNodePath.getValue();
                sofa::helper::vector<std::string> toolNodePathV = toolNodePath.getValue();
                
                if (armatureEndNodePathV.size() == toolNodePathV.size())
                {
                    armatureEndStateV.resize(armatureEndNodePathV.size());
                    oldPosV.resize(armatureEndNodePathV.size());
                    toolStateV.resize(armatureEndNodePathV.size());

                    canHandleTools = true;
                    for (unsigned int stateI = 0; stateI < armatureEndNodePathV.size(); stateI++)
                    {

                        // <!-- TODO: replace this with the proper SOFA way to do this 
                        msg_info("ZyColladaToolHandler") << "armatureEndNodePath: " << armatureEndNodePathV.at(stateI);
                        msg_info("ZyColladaToolHandler") << "toolNodePath: " << toolNodePathV.at(stateI);
                        {
                            std::istringstream readPathStream(armatureEndNodePathV.at(stateI));

                            simulation::Node* armatureEndNode = dynamic_cast<simulation::Node*>(getContext()->getRootContext());

                            while (!readPathStream.eof())
                            {
                                std::string tmp;
                                std::getline(readPathStream, tmp, '/');

                                armatureEndNode = armatureEndNode->getChild(tmp); // TODO: find a way to do this, so that SOFA doesn't crash when the path is wrong
                            }

                            if (armatureEndNode)
                            {
                                armatureEndStateV.at(stateI) = dynamic_cast<sofa::component::container::MechanicalObject<DataTypes>*>(armatureEndNode->getMechanicalState());
                                if (armatureEndStateV.at(stateI))
                                {
                                    oldPosV.at(stateI) = armatureEndStateV.at(stateI)->getPosition();
                                }
                            }
                        }

                        {
                            std::istringstream readPathStream(toolNodePathV.at(stateI));

                            simulation::Node* toolNode = dynamic_cast<simulation::Node*>(getContext()->getRootContext());

                            while (!readPathStream.eof())
                            {
                                std::string tmp;
                                std::getline(readPathStream, tmp, '/');

                                toolNode = toolNode->getChild(tmp); // TODO: find a way to do this, so that SOFA doesn't crash when the path is wrong
                            }

                            if (toolNode)
                            {
                                toolStateV.at(stateI) = dynamic_cast<sofa::component::container::MechanicalObject<DataTypes>*>(toolNode->getMechanicalState());
                            }
                        }

                        if (armatureEndStateV.at(stateI))
                        {
                            msg_info("ZyColladaToolHandler") << "End of kinematics chain (tool attachment point): " << armatureEndStateV.at(stateI)->getName();
                        }
                        else
                        {
                            msg_warning("ZyColladaToolHandler") << "End of kinematics chain (tool attachment point) with path \"" << armatureEndNodePath.getValue().at(stateI) << "\" not found.";
                        }

                        if (toolStateV.at(stateI))
                        {
                            msg_info("ZyColladaToolHandler") << "Attached tool: " << toolStateV.at(stateI)->getName();
                        }
                        else
                        {
                            msg_warning("ZyColladaToolHandler") << "Attached tool with path \"" << toolNodePath.getValue().at(stateI) << "\" not found.";
                        }
                    }
                }
                else
                {
                    canHandleTools = false; 
                    msg_error("ZyColladaToolHandler") << "Not the same number of armatureEndNodePaths (" << armatureEndNodePathV.size() << " given) and toolNodePaths (" << toolNodePathV.size() << " given)!";
                }
            }

            template<class DataTypes>
            void ZyColladaToolHandler<DataTypes>::handleTool(simulation::common::MechanicalOperations& mop, sofa::core::behavior::MultiVecCoord& pos/*, sofa::core::behavior::MultiVecCoord& freePos*/)
            {
                if (canHandleTools)
                {
                    mop.propagateX(pos);

                    for (unsigned int stateI = 0; stateI < armatureEndStateV.size(); stateI++)
                    {
                        if (armatureEndStateV.at(stateI) && toolStateV.at(stateI))
                        {
                            VecCoord curPos = toolStateV.at(stateI)->getPosition();
                            VecCoord newPos = armatureEndStateV.at(stateI)->getPosition();

                            VecCoord posChange = newPos;
                            double posChangeAcc = 0.0;
                            for (unsigned int i = 0; i < newPos[0].size(); i++)
                            {
                                posChange[0][i] = newPos[0][i] - oldPosV.at(stateI)[0][i];
                                posChangeAcc += (posChange[0][i] * posChange[0][i]);
                            }

                            if (posChangeAcc > 0.000000000001)
                            {
                                for (unsigned int j = 0; j < curPos.size(); j++)
                                {
                                    Coord tmp = curPos[j];
                                    for (unsigned int k = 0; k < posChange[0].size(); k++)
                                    {
                                        curPos[j][k] = curPos[j][k] + posChange[0][k];
                                    }
                                }
                            }

                            toolStateV.at(stateI)->setPosition(curPos);

                            oldPosV.at(stateI) = newPos;
                        }
                    }
                
                    mop.propagateX(pos);
                }

                return;
            }
            
        } // namespace controller

    } // namespace component

} // namespace sofa

#endif
