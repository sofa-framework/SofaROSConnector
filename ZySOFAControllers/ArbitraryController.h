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
// C++ Implementation: ArbitraryController
//
//
#ifndef SOFA_COMPONENT_CONTROLLER_ArbitraryController_H
#define SOFA_COMPONENT_CONTROLLER_ArbitraryController_H

#include <SofaUserInteraction/ArticulatedHierarchyController.h>
#include <sofa/helper/io/bvh/BVHLoader.h>
#include <sofa/defaulttype/Vec.h>

#include <sofa/helper/map.h>

namespace sofa
{

namespace component
{

namespace controller
{

using namespace sofa::defaulttype;

/**
 * @brief ArbitraryController Class.
 *
 * Implements a handler that controls the values of the
 * articulations of an articulated hierarchy container.
 * .bvh files are controlling the value.
 */
class SOFA_USER_INTERACTION_API ArbitraryController : public ArticulatedHierarchyController
{
#define ZYKLIO "Zyklio"

public:
    SOFA_CLASS(ArbitraryController,ArticulatedHierarchyController);

protected:
    /**
    * @brief Default Constructor.
     */
    ArbitraryController()
        : kinValues(initData(&kinValues, "KinematicValues", "Value of each degree of freedom"))
        , minValues(initData(&minValues, "MinimumValues", "Minimum values of each degree of freedom"))
        , maxValues(initData(&maxValues, "MaximumValues", "Maximum values of each degree of freedom"))
        , invertAxis(initData(&invertAxis, "InvertAxis", "True for each inverted axis, false otherwise"))
        , controlIndex(initData(&controlIndex, "ControlIndex", "Min and Max value of controlled joint indices."))
        , jointNames(initData(&jointNames, "JointNames", "Name of the joint"))
        //
        , jointControlledByROS(initData(&jointControlledByROS, "controlledByROS", "If true, corresponding joint values are set by the ROS connector."))
    {
        initialized = false;
        // Setup groups
        kinValues.setGroup(ZYKLIO);
        minValues.setGroup(ZYKLIO);
        maxValues.setGroup(ZYKLIO);
        invertAxis.setGroup(ZYKLIO);
        jointNames.setGroup(ZYKLIO);

        // Setup Default Values
        this->f_listening.setValue(true);
        controlIndex.setValue(Vec2i(3, 8)); // TODO: put sensible values here; this is hard-coded for Cebit Demo (Cebit 2016, in case you read this in 2017 or later)
        //controlIndex.setValue(Vec2i(3, 40)); // now it's hard-coded for the SCHUNK-Demo / Automatica (2016)
                                            // TODO there is a corresponding "arbitraryControl->setControlIndex(3,8);" in SceneColladaLoader.cpp


        // don't show some items
        angleDelta.setDisplayed(false);
        propagateUserInteraction.setDisplayed(false);
        articulationsIndices.setDisplayed(false);
    }

    /**
     * @brief Default Destructor.
     */
    virtual ~ArbitraryController() {}

public:
    typedef sofa::helper::vector<double> doubleVector;
    typedef sofa::helper::vector<bool> boolVector;
    typedef sofa::helper::vector<std::string> stringVector;


    /**
     * @brief Init method called during the scene graph initialization.
     */
    virtual void init();
    // <!-- untested Cebit 2016 code
    /*void bwdInit();*/
    /*Data<std::string> gripperNodePath1;*/ // TODO: replace this with the proper SOFA way to do this
    /*Data<std::string> gripperNodePath2;*/ // TODO: replace this with the proper SOFA way to do this
    /*sofa::component::mapping::RigidMapping<Rigid3dTypes, Vec3dTypes>* gripperMap1;
    sofa::component::mapping::RigidMapping<Rigid3dTypes, Vec3dTypes>* gripperMap2;*/
    // -->

    /**
     * @brief Reset to initial state
     */
    virtual void reset();

    /**
     * @brief Apply the controller current modifications to its controled component.
     */
    virtual void applyController(void);

    void setJointNames(std::vector<std::string> jointNames);
    void setKinValues(std::vector<double> kinValues);
    void setMinValues(std::vector<double> minValues);
    void setMaxValues(std::vector<double> maxValues);
    void setInvValues(std::vector<bool> invertAxis);

    void initJointControlledByROS(unsigned int size);

    template<class T> // T= double, bool, ...
    void setValuesFromVector(Data<sofa::helper::vector<T> >  &v, std::vector<T> values)
    {
        sofa::helper::vector<T> * val = v.beginEdit();
        val->resize(values.size());
        for (unsigned int i = 0; i < values.size(); i++) {
            (*val)[i] = values[i];
        }
        v.endEdit();
    }

    int getMinIndex() {return controlIndex.getValue().x();}
    int getMaxIndex() {return controlIndex.getValue().y();}
    void setControlIndex(int min, int max);
    void setToolValues(std::vector<double> values);
    void setToolValue(double value, int index);
    void setRotValueRad(double value, int index);
    void setRotValueRadByName(double value, std::string name);

protected:

    Data< stringVector >   jointNames;
    Data< doubleVector >  kinValues;
    Data< doubleVector >  minValues;
    Data< doubleVector >  maxValues;
    Data< boolVector >    invertAxis;
    Data< Vec2i >         controlIndex;
    ArtCenterVec m_artCenterVec; ///< List of ArticulationCenters controlled by the controller.
    ArticulatedHierarchyContainer* ahc;
    bool initialized;

    // for interaction with ROS:
    //      corresponds to jointNames. If jointControlledByROS[k]=true, then jointNames[k] is controlled by
    //      the ROSConnector, otherwise it's not (making it possible to control it otherwise).
    Data < boolVector > jointControlledByROS;

};

} // namespace controller

} // namespace component

} // namespace sofa

#endif //SOFA_COMPONENT_CONTROLLER_ArbitraryController_H
