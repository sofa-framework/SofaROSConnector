cmake_minimum_required(VERSION 2.8.12)

project(ZyColladaLoader)

set(HEADER_FILES
	ColladaTransformHelper.h
        ZyColladaLoader.h
	config.h
)

set(SOURCE_FILES
        ColladaTransformHelper.cpp
        ZyColladaLoader.cpp
        initPlugin.cpp
)

set(README_FILES ColladaSceneLoader.txt)

find_package(SofaBase REQUIRED)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})
find_package(Flexible QUIET)
find_package(image QUIET)

add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES} ${README_FILES})
target_link_libraries(${PROJECT_NAME} SofaLoader SofaSimulationCommon SofaBaseMechanics SofaRigid SofaBoundaryCondition SofaBaseTopology SofaBaseVisual SofaOpenglVisual SofaDeformable SofaMeshCollision SofaGuiQt)

if ("${CMAKE_BUILD_TYPE}" STREQUAL "Debug")
    target_link_libraries(${PROJECT_NAME} assimpD)
else()
    target_link_libraries(${PROJECT_NAME} assimp)
endif()

set(QT5_NO_LINK_QTMAIN 1)

find_package(Qt5 QUIET COMPONENTS Core Gui Widgets)
if(Qt5Core_FOUND)
    message(STATUS "${PROJECT_NAME}: Using Qt5")
	include_directories(${Qt5Core_INCLUDE_DIRS} ${Qt5Widgets_INCLUDE_DIRS} ${Qt5OpenGL_INCLUDE_DIRS})
	set(EXT_QT_LIBRARIES ${Qt5Core_LIBRARIES} ${Qt5Widgets_LIBRARIES} ${Qt5OpenGL_LIBRARIES})
else()
    message(STATUS "${PROJECT_NAME}: Qt5 not found, using Qt4")

    set(QT4_NO_LINK_QTMAIN 1)
    find_package(Qt4 QUIET COMPONENTS qtcore qtgui REQUIRED)
    include_directories(${QT_INCLUDE_DIRS})

	set(EXT_QT_LIBRARIES ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY} ${QT_QTOPENGL_LIBRARY} ${QT_QTXML_LIBRARY})
endif()

target_link_libraries(${PROJECT_NAME} ${EXT_QT_LIBRARIES})

find_package(Boost COMPONENTS regex REQUIRED)
include_directories(${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIRS})

if(SOFA_PLUGIN_OBBTREEGPU_ENABLED)
	add_definitions("-DSOFA_HAVE_PLUGIN_OBBTREEGPU")
	# target_link_libraries(${PROJECT_NAME} ObbTreeGPU PQP)
	# include_directories(
	#	${CMAKE_SOURCE_DIR}/truplugins/ObbTreeGPU
	#	${CMAKE_SOURCE_DIR}/trulibs/PQP
	#	${CMAKE_SOURCE_DIR}/trulibs/PQP/src
	#	${CMAKE_SOURCE_DIR}/trulibs/PQP/include
	#	${CMAKE_SOURCE_DIR}/trulibs/gProximity
	#	${CMAKE_SOURCE_DIR}/trulibs/CyberX3D
	# )	
	
	#if(SOFA-EXTERNAL_GLEW)
#		add_definitions("-DSOFA_HAVE_GLEW")
#	endif()
	
	if(WIN32)
		# include_directories(${CMAKE_SOURCE_DIR}/truplugins/RobotConnector)
		#link_directories(${CMAKE_SOURCE_DIR}/lib/Win32)
		# link_directories(${CMAKE_SOURCE_DIR}/lib/Win64)
	endif()
endif()
#tst ende

set(ZYCOLLADALOADER_COMPILE_FLAGS "-DSOFA_BUILD_ZYCOLLADALOADER")

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/..
                    ${CMAKE_CURRENT_SOURCE_DIR}
                    ${CMAKE_SOURCE_DIR}/applications
                    ${CMAKE_SOURCE_DIR}/framework
                    ${CMAKE_SOURCE_DIR}/modules
                    ${CMAKE_BINARY_DIR}/include
                    ${CMAKE_BINARY_DIR}/SofaGui/SofaGuiQt)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../assimp-3.0.1270/include
                    ${CMAKE_CURRENT_SOURCE_DIR}/../assimp-3.0.1270/code
                   )
# FILE(TO_CMAKE_PATH "${OGRE_INCLUDE_DIR}/.." OGREINCLUDE) 
# include_directories(${OGREINCLUDE})
# message(STATUS "${PROJECT_NAME}: including ${OGRE_INCLUDE_DIR}/..")

if ("${CMAKE_BUILD_TYPE}" STREQUAL "Debug")
    target_link_libraries(${PROJECT_NAME} ${CMAKE_BINARY_DIR}/lib/libassimpD.a)
else()
    target_link_libraries(${PROJECT_NAME} ${CMAKE_BINARY_DIR}/lib/libassimp.a)
endif()
target_link_libraries(${PROJECT_NAME} hashgen)

target_link_libraries(${PROJECT_NAME} ${Boost_REGEX_LIBRARY})

if(Flexible_FOUND)
    target_link_libraries(${PROJECT_NAME} Flexible)
    set(ZYCOLLADALOADER_COMPILE_FLAGS "${TRUCOLLADALOADER_COMPILE_FLAGS} -DSOFA_HAVE_PLUGIN_FLEXIBLE")
else()
    message(STATUS "${PROJECT_NAME}: could not find Flexible")
endif()

if(image_FOUND)
    target_link_libraries(${PROJECT_NAME} image)
    set(ZYCOLLADALOADER_COMPILE_FLAGS "${TRUCOLLADALOADER_COMPILE_FLAGS} -DSOFA_HAVE_PLUGIN_IMAGE")
else()
    message(STATUS "${PROJECT_NAME}: could not find image")
endif()

add_definitions("-DSOFA_BUILD_ZYCOLLADALOADER")
target_link_libraries(${PROJECT_NAME} SofaLoader SofaSimulationCommon SofaBaseMechanics SofaRigid SofaBoundaryCondition SofaBaseTopology SofaBaseVisual SofaDeformable SofaMeshCollision SofaUserInteraction)
target_link_libraries(${PROJECT_NAME} ZySOFAControllers)

if(SOFA_PLUGIN_OBBTREEGPU_ENABLED)
    target_link_libraries(${PROJECT_NAME} SofaConstraint)
endif()

if(SOFA_PLUGIN_ROBOTCONNECTOR_ENABLED AND WIN32)
    target_link_libraries(${PROJECT_NAME} RobotConnector)
endif()

if(SOFA_PLUGIN_QTOGREVIEWER_ENABLED)
    target_link_libraries(${PROJECT_NAME} QtOgreViewer)
endif()

set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "${ZYCOLLADALOADER_COMPILE_FLAGS}")
set_target_properties(${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX "_d")

sofa_create_package(${PROJECT_NAME} "0.0.1" ${PROJECT_NAME} ${PROJECT_NAME})

install(DIRECTORY Demos/ DESTINATION share/sofa/plugins/${PROJECT_NAME})

