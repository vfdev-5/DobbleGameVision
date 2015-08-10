project( 3rdparty )

## include & link to OpenCV :
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIB_DIR})
link_libraries(${OpenCV_LIBS})

## search files:
SET(SRC_FILES "")
SET(INC_FILES "")

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
#add_definitions("-DLIB_EXPORT")
#add_library( ${PROJECT_NAME} SHARED ${SRC_FILES} ${INC_FILES} ${UI_FILES} ${ResourceSources})
#set_target_properties(${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX ".d")

## installation :
#install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION bin ARCHIVE DESTINATION lib LIBRARY DESTINATION lib)
#foreach(module ${modules})
#    INSTALL(DIRECTORY ${CMAKE_SOURCE_DIR}/Lib/${module} DESTINATION include FILES_MATCHING PATTERN #"*.h" )
#endforeach()



