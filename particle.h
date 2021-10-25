//
// Created by Leo on 2021/10/23.
//

#ifndef SIMPLE_FLUID_SIMULATOR_PARTICLE_H
#define SIMPLE_FLUID_SIMULATOR_PARTICLE_H

#include "glm/glm.hpp"
#include <cstring>

struct Particle{

public:
    glm::vec3 		pos;
    float			density;
    float			pressure;
    glm::vec3 		acceleration;
    glm::vec3 		velocity;
    int				next;
};


class ParticleBuffer{

public:
    void reset(unsigned int capacity);
    unsigned int size() const { return m_particleCounts; }
    Particle* get(unsigned int index) { return m_particleBuf+index; }
    const Particle* get(unsigned int index) const { return m_particleBuf+index; }
    Particle* AddParticle();

private:
    Particle* m_particleBuf;
    unsigned int m_particleCounts;
    unsigned int m_bufCapacity;

private:
    const unsigned int MAX_PARTICLE;
public:
    ParticleBuffer();
    virtual ~ParticleBuffer();
};


class ParticleTable{



};

#endif //SIMPLE_FLUID_SIMULATOR_PARTICLE_H
