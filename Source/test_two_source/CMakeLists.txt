## Project DoubleGameVision
project( DoubleGameVision )
cmake_minimum_required( VERSION 2.8.11 FATAL_ERROR)


# Add custom Cmake Modules
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/CMakeModules")

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)
# Instruct CMake to run uic automatically when needed.
set(CMAKE_AUTOUIC ON)


# Find OpenCV, Qt:
set(OpenCV_STATIC OFF)
find_package( OpenCV 3.0.0 REQUIRED)
set( QT_COMPONENT_LIST Widgets Test)
find_package( Qt REQUIRED )

# add library :
add_subdirectory("Lib")
add_subdirectory("App")
#add_subdirectory("Sandbox")

if(WIN32)
    ## install opencv library dlls :
    get_filename_component(OpenCV_BIN_DIR "${OpenCV_LIB_PATH}/../bin" ABSOLUTE)
#    message("OpenCV_BIN_DIR : ${OpenCV_BIN_DIR}")
    INSTALL(DIRECTORY ${OpenCV_BIN_DIR}/ DESTINATION bin CONFIGURATIONS Debug FILES_MATCHING PATTERN "*d.dll" )
    INSTALL(DIRECTORY ${OpenCV_BIN_DIR}/ DESTINATION bin CONFIGURATIONS Release FILES_MATCHING PATTERN "*.dll" PATTERN "*d.dll" EXCLUDE )
endif(WIN32)




