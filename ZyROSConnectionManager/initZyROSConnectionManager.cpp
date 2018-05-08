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
*                               SOFA :: Plugins                               *
*                                                                             *
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#include "initZyROSConnectionManager.h"

namespace Zyklio
{

    namespace  ROSConnectionManager
    {

        void initExternalModule()
        {
            static bool first = true;
            if (first)
            {
                first = false;
            }
        }

        const char* getModuleName()
        {
            return "Truphysics ROS Connection Manager";
        }

        const char* getModuleVersion()
        {
            return "0.1";
        }

        const char* getModuleLicense()
        {
            return "(c) Zyklio";
        }


        const char* getModuleDescription()
        {
            return "Zyklio components.";
        }

        const char* getModuleComponentList()
        {
            /// string containing the names of the classes provided by the plugin
            return "ZyROSConnectionManager";
        }
    }

}

/// Use the SOFA_LINK_CLASS macro for each class, to enable linking on all platforms
SOFA_LINK_CLASS(ZyROSConnectionManager)
//SOFA_LINK_CLASS(ZyROSConnector)
//SOFA_LINK_CLASS(TruRosConnectorTopicSubscriberIface)
//SOFA_LINK_CLASS(ZyROSJointStateListener)
//SOFA_LINK_CLASS(ZyROSLogListener)
