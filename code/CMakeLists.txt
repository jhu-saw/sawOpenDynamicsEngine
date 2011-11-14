
set( 
  REQUIRED_CISST_LIBRARIES
  cisstCommon
  cisstVector
  cisstNumerical
  cisstOSAbstraction
  cisstMultiTask
  cisstParameterTypes
  cisstRobot )

find_package (cisst REQUIRED ${REQUIRED_CISST_LIBRARIES})
find_package( sawOpenSceneGraph )
find_package( OpenDynamicsEngine )

if( cisst_FOUND AND sawOpenSceneGraph_FOUND AND ODE_FOUND )

  SET( 
    HEADER_FILES 
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
    )
    
  SET(
    SOURCE_FILES
    osaODEBarrettHand.cpp
    osaODEBody.cpp
    osaODEJoint.cpp
    osaODEManipulator.cpp
    osaODEServoMotor.cpp
    osaODEWorld.cpp

    mtsODEManipulatorTask.cpp
    )

  include_directories( 
    ${CISST_INCLUDE_DIR}
    ${sawOpenSceneGraph_INCLUDE_DIRS}  
    ${ODE_INCLUDE_DIR} )

  add_library( sawOpenDynamicsEngine ${HEADER_FILES} ${SOURCE_FILES} )

  set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DCISST_ODE_SUPPORT -DdDOUBLE" )

  target_link_libraries(
    sawOpenDynamicsEngine 
    ${ODE_LIBRARY} 
    ${sawOpenSceneGraph_LIBRARIES} )

  cisst_target_link_libraries( 
    sawOpenDynamicsEngine 
    ${REQUIRED_CISST_LIBRARIES} )


    set( 
      sawOpenDynamicsEngine_INCLUDE_DIR 
      "${sawComponents_SOURCE_DIR}" 
      "${sawComponents_BINARY_DIR}"
      ${ODE_INCLUDE_DIRS} )

    set(sawOpenDynamicsEngine_LIBRARY_DIR "${LIBRARY_OUTPUT_PATH}")
    set(sawOpenDynamicsEngine_LIBRARIES sawOpenDynamicsEngine ${sawOpenSceneGraph_LIBRARIES} )

    configure_file (
      "${CMAKE_CURRENT_SOURCE_DIR}/sawOpenDynamicsEngineConfig.cmake.in"
      "${sawComponents_BINARY_DIR}/sawOpenDynamicsEngineConfig.cmake"
      @ONLY@)


endif( cisst_FOUND AND sawOpenSceneGraph_FOUND AND ODE_FOUND )


