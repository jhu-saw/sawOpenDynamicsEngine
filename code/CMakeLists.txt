#
# $Id$
#
# (C) Copyright 2012 Johns Hopkins University (JHU), All Rights
# Reserved.
#
# --- begin cisst license - do not edit ---
# 
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
# 
# --- end cisst license ---

set( 
  REQUIRED_CISST_LIBRARIES
  cisstCommon
  cisstVector
  cisstNumerical
  cisstOSAbstraction
  cisstMultiTask
  cisstParameterTypes
  cisstRobot )

find_package( cisst REQUIRED ${REQUIRED_CISST_LIBRARIES} )
find_package( OpenDynamicsEngine )

if( cisst_FOUND AND ODE_FOUND )

  include( ${CISST_USE_FILE} )
  cisst_find_component( sawOpenSceneGraph REQUIRED  )

  if( sawOpenSceneGraph_FOUND )

    SET( HEADER_FILES 
      ../osaODEBarrettHand.h
      ../osaODEBody.h
      ../osaODEJoint.h
      ../osaODEManipulator.h
      ../osaODEServoMotor.h
      ../osaODEWorld.h

      ../mtsODEManipulatorTask.h
      ../mtsODEWorld.h

      ../mtsODEManipulator.h
      ../mtsODEBarrettHand.h

      ../sawOpenDynamicsEngineExport.h
      )

    SET( SOURCE_FILES
      osaODEBarrettHand.cpp
      osaODEBody.cpp
      osaODEJoint.cpp
      osaODEManipulator.cpp
      osaODEServoMotor.cpp
      osaODEWorld.cpp

      mtsODEManipulatorTask.cpp
      )

    include_directories( 
      ${sawOpenSceneGraph_INCLUDE_DIRS}  
      ${OpenDynamicsEngine_INCLUDE_DIRS} )

    add_library( sawOpenDynamicsEngine ${SOURCE_FILES} )

    set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DCISST_ODE_SUPPORT -DdDOUBLE" )

    target_link_libraries(
      sawOpenDynamicsEngine 
      ${OpenDynamicsEngine_LIBRARIES} 
      ${sawOpenSceneGraph_LIBRARIES} )

    cisst_target_link_libraries( 
      sawOpenDynamicsEngine 
      ${REQUIRED_CISST_LIBRARIES} )

    set( 
      sawOpenDynamicsEngine_INCLUDE_DIR 
      ${sawComponents_SOURCE_DIR} 
      ${sawComponents_BINARY_DIR}
      ${OpenDynamicsEngine_INCLUDE_DIRS} )

    set(sawOpenDynamicsEngine_LIBRARY_DIR "${LIBRARY_OUTPUT_PATH}")
    set(sawOpenDynamicsEngine_LIBRARIES sawOpenDynamicsEngine ${sawOpenSceneGraph_LIBRARIES} )
    set(sawOpenDynamicsEngine_DEFINITIONS "-DdDOUBLE" )

    configure_file (
      "${CMAKE_CURRENT_SOURCE_DIR}/sawOpenDynamicsEngineConfig.cmake.in"
      "${sawComponents_BINARY_DIR}/sawOpenDynamicsEngineConfig.cmake"
      @ONLY@)

    # Install target for headers and library
    install (FILES ${HEADER_FILES}
      DESTINATION include/sawOpenDynamicsEngine)

    install (TARGETS sawOpenDynamicsEngine
      RUNTIME DESTINATION bin
      LIBRARY DESTINATION lib
      ARCHIVE DESTINATION lib)

    # Install cmake files for finding ODE and sawODE
    install (FILES "${sawComponents_BINARY_DIR}/sawOpenDynamicsEngineConfig.cmake"
      DESTINATION ${CISST_CMAKE_MODULES_INSTALL_SUFFIX}/components)

    install (FILES "${sawOpenDynamicsEngine_SOURCE_DIR}/code/cmake/Modules/FindOpenDynamicsEngine.cmake"
      DESTINATION ${CISST_CMAKE_MODULES_INSTALL_SUFFIX})

  endif( sawOpenSceneGraph_FOUND )
endif( cisst_FOUND AND ODE_FOUND )


