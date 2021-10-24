//
// Created by Leo on 2021/10/24.
//

#ifndef SIMPLE_FLUID_SIMULATOR_SPH_SYSTEM_H
#define SIMPLE_FLUID_SIMULATOR_SPH_SYSTEM_H

#include "particle_box.h"

class SPHSystem{

public:
     void init(unsigned short maxPointCounts,
                      const glm::vec3 wallBox_min, const glm::vec3 wallBox_max,
                      const glm::vec3 initFluidBox_min, const glm::vec3 initFluidBox_max,
                      const glm::vec3 gravity)
    {

        _init(maxPointCounts,
              ParticleBox3(wallBox_min, wallBox_max),
              ParticleBox3(initFluidBox_min, initFluidBox_max),
              gravity);
    }

    unsigned int getPointStride() const { return sizeof(Particle); }
    unsigned int getPointCounts() const { return m_pointBuffer.size(); }
    const glm::vec3* getPointBuf() const { return (const glm::vec3*)m_pointBuffer.get(0); }
    virtual void tick();

private:

    void _init(unsigned short maxPointCounts, const ParticleBox3& wallBox, const ParticleBox3& initFluidBox, const glm::vec3 & gravity);
    void _computePressure();
    void _computeForce();
    void _advance();
    void _addFluidVolume(const ParticleBox3& fluidBox, float spacing);

private:
    ParticleBuffer m_pointBuffer;
    ParticleGridContainer m_gridContainer;
    NeighborTable m_neighborTable;

    // SPH Kernel
    float m_kernelPoly6;
    float m_kernelSpiky;
    float m_kernelViscosity;

    //Other Parameters
    float m_unitScale;
    float m_viscosity;
    float m_restDensity;
    float m_pointMass;
    float m_smoothRadius;
    float m_gasConstantK;
    float m_boundartStiffness;
    float m_boundaryDampening;
    float m_speedLimiting;
    glm::vec3 m_gravityDir;

    ParticleBox3 m_sphWallBox;
public:
    SPHSystem();
    ~SPHSystem();

};


#endif //SIMPLE_FLUID_SIMULATOR_SPH_SYSTEM_H
