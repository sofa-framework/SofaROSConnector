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
//
// C++ Implementation: RobotController
//
//
#ifndef SOFA_COMPONENT_CONTROLLER_ROBOTCONTROLLER_H
#define SOFA_COMPONENT_CONTROLLER_ROBOTCONTROLLER_H

#include <SofaUserInteraction/ArticulatedHierarchyController.h>
#include <sofa/helper/io/bvh/BVHLoader.h>
#include <sofa/defaulttype/Vec.h>

namespace sofa
{

namespace component
{

namespace controller
{

using namespace sofa::defaulttype;

/**
 * @brief RobotController Class.
 *
 * Implements a handler that controls the values of the
 * articulations of an articulated hierarchy container.
 * .bvh files are controlling the value.
 */
class SOFA_USER_INTERACTION_API RobotController : public ArticulatedHierarchyController
{
public:
    SOFA_CLASS(RobotController,ArticulatedHierarchyController);

protected:
    /**
    * @brief Default Constructor.
     */
    RobotController()
        : rotValues(initData(&rotValues, "RotationValues", "DOF values for robot axes 1-6 in deg"))
        , posValues(initData(&posValues, "RobotPosition", "Position values for robot base x,y,z"))
        , rotOffsetValues(initData(&rotOffsetValues, "Offset", "Apply offset on specific axes"))
        , rotDirection(initData(&rotDirection, "InvertRotationDirection", "Invert the rotation for specific axes (1=invert 0=normal)"))
    {
        // Setup groups
        rotValues.setGroup("Zyklio");
        posValues.setGroup("Zyklio");
        rotOffsetValues.setGroup("Zyklio");
        rotDirection.setGroup("Zyklio");

        // Setup Default Values
        this->f_listening.setValue(false);

        /*Vec6d * rot = rotOffsetValues.beginEdit();
        (*rot)[1] = 90.0;
        (*rot)[2] = -90.0;
        rotOffsetValues.endEdit();*/

        // Seems lika all Yrotation Axes have to be inverted ... could be done automatically
        /*Vec6b * dir = rotDirection.beginEdit();
        (*dir)[1] = 1;
        (*dir)[2] = 1;
        (*dir)[4] = 1;
        rotDirection.endEdit();*/

        Vec<6, bool> * dir = rotDirection.beginEdit();
        (*dir)[1] = 1;
        rotDirection.endEdit();

        // don't show some items
        angleDelta.setDisplayed(false);
        propagateUserInteraction.setDisplayed(false);
        articulationsIndices.setDisplayed(false);
    }

    /**
     * @brief Default Destructor.
     */
    virtual ~RobotController() {}

public:
    /**
     * @brief Init method called during the scene graph initialization.
     */
    virtual void init();

    /**
     * @brief Reset to initial state
     */
    virtual void reset();

    /**
     * @brief Apply the controller current modifications to its controled component.
     */
    virtual void applyController(void);

    /**
     * @brief Set the rotation values of the axes
     * @param num    Number of axes
     * @param value  List with rotation values
     */
    void setRotValueRad(int num, double* value);
    
protected:
    Data< Vec6d > rotValues;
    Data< Vec3d > posValues;
    Data< Vec6d > rotOffsetValues;
    Data< Vec<6, bool> > rotDirection;
    ArtCenterVec m_artCenterVec; ///< List of ArticulationCenters controlled by the controller.
    ArticulatedHierarchyContainer* ahc;

    Vec6d newRotValues;
    bool hasNewRotValues;
};

} // namespace controller

} // namespace component

} // namespace sofa

#endif //SOFA_COMPONENT_CONTROLLER_ROBOTCONTROLLER_H
