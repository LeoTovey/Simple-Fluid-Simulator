
set(STB_SOURCE_FILES stb_image.cpp)

set(STB_HEADER_FILES stb_image.h)

add_library(stb_image STATIC ${STB_SOURCE_FILES} ${STB_HEADER_FILES})

target_include_directories(stb_image PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} FILES ${STB_SOURCE_FILES} ${STB_HEADER_FILES})