#
# $Id$
#
# (C) Copyright 2009 Johns Hopkins University (JHU), All Rights
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
  cisstRobot
  cisstParameterTypes )

find_package( cisst REQUIRED ${REQUIRED_CISST_LIBRARIES} )
find_package( sawOpenDynamicsEngine REQUIRED HINTS "${saw_BINARY_DIR}/components" )

if( cisst_FOUND_AS_REQUIRED AND sawOpenDynamicsEngine_FOUND )

  include( ${CISST_USE_FILE} )

  include_directories( ${sawOpenDynamicsEngine_INCLUDE_DIR} )
  link_directories( ${sawOpenDynamicsEngine_LIBRARY_DIR} )
  set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DCISST_ODE_SUPPORT -DdDOUBLE" )

  add_executable( osaODEBodyExample           osaODEBodyExample.cpp )
  add_executable( osaODEWAMExample            osaODEWAMExample.cpp )
  add_executable( osaODEWAMBarrettHandExample osaODEWAMBarrettHandExample.cpp )
  add_executable( osaODEBarrettHandExample    osaODEBarrettHandExample.cpp )

  add_executable( mtsODEWAMExample            mtsODEWAMExample.cpp )
  add_executable( mtsODEWAMBarrettHandExample mtsODEWAMBarrettHandExample.cpp )
  add_executable( mtsODEBarrettHandExample    mtsODEBarrettHandExample.cpp )

  target_link_libraries( osaODEBodyExample           sawOpenDynamicsEngine ) 
  target_link_libraries( osaODEWAMExample            sawOpenDynamicsEngine ) 
  target_link_libraries( osaODEWAMBarrettHandExample sawOpenDynamicsEngine ) 
  target_link_libraries( osaODEBarrettHandExample    sawOpenDynamicsEngine ) 

  target_link_libraries( mtsODEWAMExample            sawOpenDynamicsEngine ) 
  target_link_libraries( mtsODEWAMBarrettHandExample sawOpenDynamicsEngine ) 
  target_link_libraries( mtsODEBarrettHandExample    sawOpenDynamicsEngine ) 

  cisst_target_link_libraries( osaODEBodyExample           ${REQUIRED_CISST_LIBRARIES} ) 
  cisst_target_link_libraries( osaODEWAMExample            ${REQUIRED_CISST_LIBRARIES} ) 
  cisst_target_link_libraries( osaODEWAMBarrettHandExample ${REQUIRED_CISST_LIBRARIES} ) 
  cisst_target_link_libraries( osaODEBarrettHandExample    ${REQUIRED_CISST_LIBRARIES} ) 

  cisst_target_link_libraries( mtsODEWAMExample            ${REQUIRED_CISST_LIBRARIES} ) 
  cisst_target_link_libraries( mtsODEWAMBarrettHandExample ${REQUIRED_CISST_LIBRARIES} ) 
  cisst_target_link_libraries( mtsODEBarrettHandExample    ${REQUIRED_CISST_LIBRARIES} ) 

endif( cisst_FOUND_AS_REQUIRED AND sawOpenDynamicsEngine_FOUND )
