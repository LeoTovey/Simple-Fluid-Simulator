cmake_minimum_required(VERSION 3.17)
project(Simple_Fluid_Simulator)

set(CMAKE_CXX_STANDARD 14)

find_package(OpenGL REQUIRED)

add_subdirectory(external/glad)
add_subdirectory(external/glfw)
add_subdirectory(external/imgui)
add_subdirectory(external/stb_image)
add_subdirectory(external/assimp)
add_subdirectory(external/glm)

include_directories(
        rendering/
        external/assimp/include/
        external/stb_image/
        external/
)

set(ALL_LIBS ${OPENGL_LIBRARY} glm glfw imgui glad stb_image assimp)

add_executable(Simple_Fluid_Simulator main.cpp particle_box.h particle_box.cpp particle.h particle.cpp sph_system.cpp sph_system.h rendering/mesh.h rendering/model.h rendering/shader.h rendering/camera.h time_integrator.cpp time_integrator.h)
target_include_directories(Simple_Fluid_Simulator PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(${PROJECT_NAME} ${ALL_LIBS})

add_definitions(-D IMGUI_IMPL_OPENGL_LOADER_GLAD)
