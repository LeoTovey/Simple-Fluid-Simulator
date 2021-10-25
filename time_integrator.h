//
// Created by Leo on 2021/10/25.
//

#ifndef SIMPLE_FLUID_SIMULATOR_TIME_INTEGRATOR_H
#define SIMPLE_FLUID_SIMULATOR_TIME_INTEGRATOR_H
#include "particle.h"

class TimeIntegrator{

public:
    explicit TimeIntegrator(float dt) :m_dt(dt) {}

    virtual void update(Particle* particle) = 0;
protected:
    float m_dt;
};

class LeapFrogIntegrator : public TimeIntegrator
{
public:
    explicit LeapFrogIntegrator(float dt) : TimeIntegrator(dt) {}

    void update(Particle* particle) override
    {
        //        // Leapfrog Integration ----------------------------
//        glm::vec3 vnext = p->velocity + accel * deltaTime;			// v(t+1/2) = v(t-1/2) + a(t) dt
//        p->velocity_eval = (p->velocity + vnext) * 0.5f;				// v(t+1) = [v(t-1/2) + v(t+1/2)] * 0.5		used to compute forces later
//        p->velocity = vnext;
//        p->pos += vnext*deltaTime/m_unitScale;		// p(t+1) = p(t) + v(t+1/2) dt
        // Leapfrog Integration ----------------------------
        glm::vec3 vnext = particle->velocity + particle->acceleration * m_dt;	// v(t+1/2) = v(t-1/2) + a(t) dt
        //particle->velocity_eval = (particle->velocity + vnext) * 0.5f;			// v(t+1) = [v(t-1/2) + v(t+1/2)] * 0.5		used to compute forces later
        particle->velocity = vnext;
        particle->pos += vnext * m_dt / 0.004f;		                            // p(t+1) = p(t) + v(t+1/2) dt
    }
};

class SemiImplicitEuler : public TimeIntegrator
{
public:
    SemiImplicitEuler(float dt) : TimeIntegrator(dt) {}

    void update(Particle *particle) override
    {
        particle->velocity = particle->velocity + particle->acceleration * m_dt;
        particle->pos += particle->velocity * m_dt / 0.004f;
    }
};

#endif //SIMPLE_FLUID_SIMULATOR_TIME_INTEGRATOR_H
