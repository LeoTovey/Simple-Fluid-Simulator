//
// Created by Leo on 2021/10/23.
//

#include "particle.h"

ParticleBuffer::ParticleBuffer():
m_particleBuf(nullptr),
m_particleCounts(0),
m_bufCapacity(0),
MAX_PARTICLE(4096)
{

}

ParticleBuffer::~ParticleBuffer()
{
    free(m_particleBuf);
    m_particleBuf = nullptr;
}

void ParticleBuffer::reset(unsigned int capacity)
{
    m_bufCapacity = capacity;
    if (m_particleBuf)
    {
        free(m_particleBuf);
        m_particleBuf = nullptr;
    }

    if (m_bufCapacity > 0)
    {
        m_particleBuf = (Particle*)malloc(m_bufCapacity* sizeof(Particle));
    }
    m_particleCounts = 0;
}

Particle *ParticleBuffer::AddParticle()
{
    if (m_particleCounts >= m_bufCapacity)
    {
        if(m_bufCapacity * 2 > MAX_PARTICLE)
        {
            //get a random point
            unsigned int index = rand() % m_particleCounts;
            return m_particleBuf+index;
        }

        //reallocate particle buffer
        m_bufCapacity *= 2;
        Particle* new_data = (Particle*)malloc(m_bufCapacity * sizeof(Particle));
        memcpy(new_data, m_particleBuf, m_particleCounts * sizeof(Particle));
        free(m_particleBuf);
        m_particleBuf = new_data;
    }

    //a new point
    Particle* particle = m_particleBuf + (m_particleCounts++);

    particle->pos = glm::vec3(0, 0, 0);
    particle->next = 0;
    particle->velocity = glm::vec3(0,0,0);
    particle->velocity_eval = glm::vec3(0,0,0);
    particle->pressure = 0;
    particle->density = 0;
    particle->acceleration = glm::vec3(0,0,0);
    return particle;
}

