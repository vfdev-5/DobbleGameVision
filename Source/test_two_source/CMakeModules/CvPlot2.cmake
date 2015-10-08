#############################################################
## CvPlot2 CMakeImport file
#############################################################

# Include directories
#############################################################

INCLUDE_DIRECTORIES(${CVPLOT2_INCLUDE_DIRS})

# Link libraries
##############################################################

LINK_DIRECTORIES(${CVPLOT2_LIBRARY_DIRS})

#IF(CMAKE_BUILD_TYPE MATCHES "Debug")
#    LINK_LIBRARIES(cvplot.d)
#ELSE()

LINK_LIBRARIES(cvplot)

#ENDIF()

# installation
#IF(WIN32)
#    INSTALL(FILES ${ShipDetectionLib_BIN_DIR}/sdCore${CMAKE_SHARED_LIBRARY_SUFFIX} DESTINATION Bin CONFIGURATIONS Release)
#    INSTALL(FILES ${ShipDetectionLib_BIN_DIR}/sdCore.d${CMAKE_SHARED_LIBRARY_SUFFIX} DESTINATION Bin CONFIGURATIONS Debug)
#    if(EXISTS ${ShipDetectionLib_BIN_DIR}/vcomp90${CMAKE_SHARED_LIBRARY_SUFFIX})
#        INSTALL(FILES ${ShipDetectionLib_BIN_DIR}/vcomp90${CMAKE_SHARED_LIBRARY_SUFFIX} DESTINATION Bin)
#    endif()
#ENDIF ()
