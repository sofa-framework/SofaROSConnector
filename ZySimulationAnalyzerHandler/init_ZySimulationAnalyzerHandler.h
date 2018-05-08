#ifndef SOFA_ZY_SIMULATION_ANALYSIS_HANDLER_CONFIG_H
#define SOFA_ZY_SIMULATION_ANALYSIS_HANDLER_CONFIG_H

#include <sofa/helper/system/config.h>

#ifdef BUILD_SOFA_ZY_SIMULATION_ANALYSIS_HANDLER
#  define SOFA_ZY_SIMULATION_ANALYSIS_HANDLER_API SOFA_EXPORT_DYNAMIC_LIBRARY
#else
#  define SOFA_ZY_SIMULATION_ANALYSIS_HANDLER_API SOFA_IMPORT_DYNAMIC_LIBRARY
#endif

#endif // SOFA_ZY_SIMULATION_ANALYSIS_HANDLER_CONFIG_H
