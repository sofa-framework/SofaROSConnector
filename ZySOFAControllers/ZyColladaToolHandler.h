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
#ifndef SOFA_COMPONENT_INTERACTIONFORCEFIELD_ZyColladaToolHandler_H
#define SOFA_COMPONENT_INTERACTIONFORCEFIELD_ZyColladaToolHandler_H

#include "initZySOFAControllers.h"

#include <sofa/core/objectmodel/Base.h>
#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/core/objectmodel/BaseObjectDescription.h>
#include <sofa/core/objectmodel/BaseContext.h>

#include <sofa/simulation/Node.h>

#include <SofaBaseMechanics/MechanicalObject.h>
#include <sofa/simulation/MechanicalOperations.h>
#include <sofa/simulation/VectorOperations.h>

#include <sofa/core/behavior/MultiVec.h>

#include <RobotController.h>

namespace sofa
{

    namespace component
    {

        namespace controller
        {

            template<class DataTypes>
            class ZyColladaToolHandler: public core::objectmodel::BaseObject
            {
            public:
                SOFA_CLASS(SOFA_TEMPLATE(ZyColladaToolHandler, DataTypes), core::objectmodel::BaseObject);

                typedef core::objectmodel::BaseObject Inherit;
                typedef typename DataTypes::VecCoord VecCoord;
                typedef typename DataTypes::VecDeriv VecDeriv;
                typedef typename DataTypes::Coord Coord;
                typedef typename DataTypes::Deriv Deriv;
                typedef typename Coord::value_type Real;
                typedef core::objectmodel::Data<VecCoord> DataVecCoord;
                typedef core::objectmodel::Data<VecDeriv> DataVecDeriv;

                typedef typename DataTypes::CPos CPos;
                typedef typename DataTypes::DPos DPos;

                typedef sofa::core::objectmodel::BaseContext BaseContext;
                typedef sofa::core::objectmodel::BaseObjectDescription BaseObjectDescription;

                virtual std::string getTemplateName() const
                {
                    return templateName(this);
                }

                static std::string templateName(const ZyColladaToolHandler<DataTypes>* = NULL)
                {
                    return DataTypes::Name();
                }

            public:

                ZyColladaToolHandler();
                ~ZyColladaToolHandler() {}

                void handleTool(simulation::common::MechanicalOperations& mop, sofa::core::behavior::MultiVecCoord& pos/*, sofa::core::behavior::MultiVecCoord& freePos*/);
                
                void init();
                void bwdInit();

                Data< sofa::helper::vector<std::string> > armatureEndNodePath; // TODO: replace this with the proper SOFA way to do this
                Data< sofa::helper::vector<std::string> > toolNodePath; // TODO: replace this with the proper SOFA way to do this

            private:
                sofa::helper::vector < sofa::component::container::MechanicalObject<DataTypes>* > armatureEndStateV; // TODO: Now that I think about it - this should be an std::pair, to make sure that the correct armature and tool states are combined
                sofa::helper::vector < sofa::component::container::MechanicalObject<DataTypes>* > toolStateV; 
                sofa::helper::vector < VecCoord > oldPosV;
                bool canHandleTools;
            };


        } // namespace controller

    } // namespace component

} // namespace sofa

#endif
