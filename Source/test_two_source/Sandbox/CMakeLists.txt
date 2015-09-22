

# add subdirectories
if(NOT WIN32)
add_definitions("-DHAS_3RDPARTY")
add_subdirectory("NonlinearDiffusionFiltering_Example")
endif()

add_subdirectory("FreqFiltering_Example")

