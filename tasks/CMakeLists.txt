# Generated from orogen/lib/orogen/templates/tasks/CMakeLists.txt

include(simulation_vrepTaskLib)
ADD_LIBRARY(${SIMULATION_VREP_TASKLIB_NAME} SHARED 
    ${SIMULATION_VREP_TASKLIB_SOURCES})
add_dependencies(${SIMULATION_VREP_TASKLIB_NAME}
    regen-typekit)



TARGET_LINK_LIBRARIES(${SIMULATION_VREP_TASKLIB_NAME}
    ${OrocosRTT_LIBRARIES}
    ${QT_LIBRARIES}
    ${SIMULATION_VREP_TASKLIB_DEPENDENT_LIBRARIES})
SET_TARGET_PROPERTIES(${SIMULATION_VREP_TASKLIB_NAME}
    PROPERTIES LINK_INTERFACE_LIBRARIES "${SIMULATION_VREP_TASKLIB_INTERFACE_LIBRARIES}")
SET_TARGET_PROPERTIES(${SIMULATION_VREP_TASKLIB_NAME}
    PROPERTIES INTERFACE_LINK_LIBRARIES "${SIMULATION_VREP_TASKLIB_INTERFACE_LIBRARIES}")

INSTALL(TARGETS ${SIMULATION_VREP_TASKLIB_NAME}
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib/orocos)

INSTALL(FILES ${SIMULATION_VREP_TASKLIB_HEADERS}
    DESTINATION include/orocos/simulation_vrep)

