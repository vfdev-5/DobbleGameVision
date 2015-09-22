project( dgv )

## include & link to OpenCV :
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIB_DIR})
link_libraries(${OpenCV_LIBS})

## include & link to Qt :
SET(INSTALL_QT_DLLS OFF)
include(Qt)

## include & link PythonLibs
find_package(PythonLibs 2 REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})
link_libraries(${PYTHON_LIBRARIES})

## include & link to project library
include_directories(${CMAKE_SOURCE_DIR}/Lib)
include_directories(${CMAKE_BINARY_DIR}/Lib)
link_directories(${CMAKE_BINARY_DIR}/Lib)
link_libraries(optimized "DGVLib" debug "DGVLib.d")


# Search source files
file(GLOB_RECURSE SRC_FILES "*.cpp")
file(GLOB_RECURSE INC_FILES "*.h")


#### for debug purposes: ################
if (NOT WIN32)
    if (CMAKE_BUILD_TYPE MATCHES Debug)
        message("Define _DEBUG symbol")
        add_definitions("-D_DEBUG")
    endif (CMAKE_BUILD_TYPE MATCHES Debug)
endif(NOT WIN32)
##########################################

## create library :
add_definitions("-DPYTHON_EXPORT")
add_library( ${PROJECT_NAME} SHARED ${SRC_FILES} ${INC_FILES})
set_target_properties(${PROJECT_NAME} PROPERTIES SUFFIX ".pyd")

## installation :
install(TARGETS ${PROJECT_NAME} DESTINATION lib)



