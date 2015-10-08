#############################################################
## CvPlot2 CMakeImport file
#############################################################

# Defines directory
#############################################################

# get path from environement variable
IF(NOT DEFINED CVPLOT2_DIR)
    SET (CVPLOT2_DIR $ENV{CVPLOT2_DIR})
ENDIF(NOT DEFINED CVPLOT2_DIR)

#IF (NOT CVPLOT2_DIR)
#    message(STATUS "CvPlot2 is not found.")
#ENDIF(NOT CVPLOT2_DIR)


if(CVPLOT2_DIR AND EXISTS ${CVPLOT2_DIR})
    #file(TO_CMAKE_PATH ${ShipDetectionLib_DIR} ShipDetectionLib_DIR)
    SET(CVPLOT2_INCLUDE_DIRS ${CVPLOT2_DIR}/include)
    SET(CVPLOT2_LIBRARY_DIRS ${CVPLOT2_DIR}/lib)
    message("cvplot2 is found")

    INCLUDE(FindPackageHandleStandardArgs)
    FIND_PACKAGE_HANDLE_STANDARD_ARGS(cvplot2 DEFAULT_MSG CVPLOT2_INCLUDE_DIRS CVPLOT2_LIBRARY_DIRS)
ENDIF(CVPLOT2_DIR AND EXISTS ${CVPLOT2_DIR})


