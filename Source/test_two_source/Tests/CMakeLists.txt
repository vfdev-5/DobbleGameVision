
## Add common files :
file(GLOB TESTS_SRC_FILES "Common.cpp")
file(GLOB TESTS_INC_FILES "Common.h")

# add subdirectories :
add_subdirectory("UnitTests/ImageCommonTest")
add_subdirectory("UnitTests/ImageProcessingTest")

