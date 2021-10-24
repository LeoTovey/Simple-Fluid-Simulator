#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <stb_image.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/string_cast.hpp>
#include <glm/gtx/norm.hpp>

#include "imgui/imgui.h"
#include "imgui/imgui_impl_glfw.h"
#include "imgui/imgui_impl_opengl3.h"

#include <shader.h>
#include <camera.h>
#include <model.h>
#include "sph_system.h"

#include <iostream>

void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void processInput(GLFWwindow *window);

// settings
const unsigned int SCR_WIDTH = 1280;
const unsigned int SCR_HEIGHT = 720;

// camera
Camera camera(glm::vec3(40.0f, 40.0f, 50.0f),glm::vec3(0.0f, 1.0f, 0.0f),-130.0f, -30.0f);
float lastX = (float)SCR_WIDTH / 2.0;
float lastY = (float)SCR_HEIGHT / 2.0;
bool firstMouse = true;

// timing
float deltaTime = 0.0f;
float lastFrame = 0.0f;

const unsigned int MAX_PARTICLE_COUNTS = 4096;
SPHSystem*				g_pSPHSystem = nullptr;
glm::vec3 			    g_wallMin{ -25, 00, -25 };
glm::vec3 			    g_wallMax{ 25, 30, 25 };

void resetSPHSystem()
{
    glm::vec3 fluid_min{ -15, 5, -15 };
    glm::vec3 fluid_max{ 15, 28, 15 };
    glm::vec3 gravity{ 0.0, -9.8f, 0 };
    g_pSPHSystem->init(MAX_PARTICLE_COUNTS, g_wallMin, g_wallMax, fluid_min, fluid_max, gravity);
}

SPHSystem* getSPHSystem()
{
    static SPHSystem s_theSystem;
    return &s_theSystem;
}

int main()
{
    // glfw: initialize and configure
    // ------------------------------
    glfwInit();
    const char* glsl_version = "#version 130";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

    // glfw window creation
    // --------------------
    GLFWwindow* window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "Simple Fluid Simulator By Long Liu", NULL, NULL);
    if (window == NULL)
    {
        std::cout << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);


    // glad: load all OpenGL function pointers
    // ---------------------------------------
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        std::cout << "Failed to initialize GLAD" << std::endl;
        return -1;
    }

    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    //io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
    //io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls

    // Setup Dear ImGui style
    ImGui::StyleColorsDark();
    //ImGui::StyleColorsClassic();

    // Setup Platform/Renderer backends
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);

    glEnable(GL_DEPTH_TEST);

    //create sph system
    g_pSPHSystem = getSPHSystem();
    resetSPHSystem();

    Shader waterParticleShader(
            "../resources/waterParticle.vs",
            "../resources/waterParticle.fs");

    Model waterParticle("../resources/water.obj");

    unsigned int amount = g_pSPHSystem->getPointCounts();
    glm::mat4* modelMatrices;
    modelMatrices = new glm::mat4[amount];
    const glm::vec3 * p = g_pSPHSystem->getPointBuf();
    float sphere_scale = 0.08f;
    for (unsigned int i = 0; i < amount; i++)
    {
        glm::mat4 model = glm::mat4(1.0f);
        model = glm::translate(model, glm::vec3(p->x,p->y,p->z));
        model = glm::scale(model, glm::vec3(sphere_scale));
        modelMatrices[i] = model;
        p = (const glm::vec3 *)(((const char*)p) + g_pSPHSystem->getPointStride());
    }


    unsigned int buffer;
    glGenBuffers(1, &buffer);
    glBindBuffer(GL_ARRAY_BUFFER, buffer);
    glBufferData(GL_ARRAY_BUFFER, amount * sizeof(glm::mat4), &modelMatrices[0], GL_DYNAMIC_DRAW);

    unsigned int VAO = waterParticle.meshes[0].VAO;
    glBindVertexArray(VAO);
    // set attribute pointers for matrix (4 times vec4)
    glEnableVertexAttribArray(3);
    glVertexAttribPointer(3, 4, GL_FLOAT, GL_FALSE, sizeof(glm::mat4), (void*)0);
    glEnableVertexAttribArray(4);
    glVertexAttribPointer(4, 4, GL_FLOAT, GL_FALSE, sizeof(glm::mat4), (void*)(sizeof(glm::vec4)));
    glEnableVertexAttribArray(5);
    glVertexAttribPointer(5, 4, GL_FLOAT, GL_FALSE, sizeof(glm::mat4), (void*)(2 * sizeof(glm::vec4)));
    glEnableVertexAttribArray(6);
    glVertexAttribPointer(6, 4, GL_FLOAT, GL_FALSE, sizeof(glm::mat4), (void*)(3 * sizeof(glm::vec4)));

    glVertexAttribDivisor(3, 1);
    glVertexAttribDivisor(4, 1);
    glVertexAttribDivisor(5, 1);
    glVertexAttribDivisor(6, 1);

    glBindVertexArray(0);

    while (!glfwWindowShouldClose(window))
    {
        g_pSPHSystem->tick();
        const glm::vec3 * p = g_pSPHSystem->getPointBuf();
        for (int i = 0; i < amount; ++i)
        {
            glm::mat4 model = glm::mat4(1.0f);
            glm::mat4 translate = glm::translate(model, *p);
            translate = glm::scale(translate,glm::vec3(sphere_scale));
            modelMatrices[i] = translate;
            p = (const glm::vec3 *)(((const char*)p) + g_pSPHSystem->getPointStride());
        }

        // Send matrix data to GPU
        glBindBuffer(GL_ARRAY_BUFFER, buffer);
        void* data = glMapBuffer(GL_ARRAY_BUFFER, GL_WRITE_ONLY);
        memcpy(data, modelMatrices, sizeof(glm::mat4) * amount);
        glUnmapBuffer(GL_ARRAY_BUFFER);
        glBindBuffer(GL_ARRAY_BUFFER, 0);

        // per-frame time logic
        // --------------------
        float currentFrame = glfwGetTime();
        deltaTime = currentFrame - lastFrame;
        lastFrame = currentFrame;

        // input
        // -----
        processInput(window);

        // render
        // ------
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // configure transformation matrices
        glm::mat4 projection = glm::perspective(glm::radians(45.0f), (float)SCR_WIDTH / (float)SCR_HEIGHT, 0.1f, 1000.0f);
        glm::mat4 view = camera.GetViewMatrix();
        waterParticleShader.use();
        waterParticleShader.setMat4("projection", projection);
        waterParticleShader.setMat4("view", view);

        glBindVertexArray(waterParticle.meshes[0].VAO);
        glDrawElementsInstanced(GL_TRIANGLES, waterParticle.meshes[0].indices.size(), GL_UNSIGNED_INT, 0, amount);
        glBindVertexArray(0);

        // Start the Dear ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        ImGui::Begin("Panel");   // Pass a pointer to our bool variable (the window will have a closing button that will clear the bool when clicked)
        ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
        if (ImGui::Button("Reset"))
        {
            resetSPHSystem();
        }
        ImGui::End();

        // Rendering
        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        // glfw: swap buffers and poll IO events (keys pressed/released, mouse moved etc.)
        // -------------------------------------------------------------------------------
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    // Cleanup
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}

// process all input: query GLFW whether relevant keys are pressed/released this frame and react accordingly
// ---------------------------------------------------------------------------------------------------------
void processInput(GLFWwindow *window)
{
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);
}

// glfw: whenever the window size changed (by OS or user resize) this callback function executes
// ---------------------------------------------------------------------------------------------
void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
    // make sure the viewport matches the new window dimensions; note that width and
    // height will be significantly larger than specified on retina displays.
    glViewport(0, 0, width, height);
}

