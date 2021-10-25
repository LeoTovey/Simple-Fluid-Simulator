//
// Created by Leo on 2021/10/24.
//

#include "sph_system.h"

#include <cmath>

SPHSystem::SPHSystem() {
    m_unitScale			= 0.004f;			// 尺寸单位
    m_viscosity			= 1.0f;				// 粘度
    m_restDensity		= 1000.f;			// 密度
    m_particleMass		= 0.0004f;			// 粒子质量
    m_gasConstantK		= 1.0f;				// 理想气体方程常量
    m_smoothRadius		= 0.01f;			// 光滑核半径

    m_boundaryStiffness = 10000.f;
    m_boundaryDampening = 256.f;
    m_speedLimiting		= 200.f;
    m_deltaTime         = 0.003f;
    m_timeIntegrator    = new SemiImplicitEuler(m_deltaTime);

    //Poly6 Kernel
    m_kernelPoly6 = 315.0f/(64.0f * 3.141592f * pow(m_smoothRadius, 9));
    //Spiky Kernel
    m_kernelSpiky = -45.0f/(3.141592f * pow(m_smoothRadius, 6));
    //Viscosity Kernel
    m_kernelViscosity = 45.0f/(3.141592f * pow(m_smoothRadius, 6));
}

SPHSystem::~SPHSystem()
{
    free(m_timeIntegrator);
}

void SPHSystem::tick()
{
    //distribute all particles to grids in gridContainer for Neighborhood Particles Search
    m_gridContainer.insertParticles(&m_particleBuffer);


    _computeDensity();
    _computeForce();
    _advance();
}

void SPHSystem::_init(unsigned short maxPointCounts,
                      const ParticleBox3 &wallBox,
                      const ParticleBox3 &initFluidBox,
                      const glm::vec3 &gravity){
    
    //allocate memory for particle buffer
    m_particleBuffer.reset(maxPointCounts);

    m_sphWallBox = wallBox;
    m_gravityDir = gravity;

    // Create particles
    float pointDistance	= std::pow(m_particleMass/m_restDensity, 1.0f/3.0f); //粒子间距
    addParticles(initFluidBox, pointDistance/m_unitScale);

    // Setup grid Grid cell size (2r)
    m_gridContainer.init(wallBox, m_unitScale, m_smoothRadius * 2.f, 1.0);
}


void SPHSystem::_computeDensity()
{
    //h^2
    float h2 = m_smoothRadius*m_smoothRadius;

    //reset neighbor table
    m_neighborTable.reset(m_particleBuffer.size());

    for(unsigned int i=0; i<m_particleBuffer.size(); i++)
    {
        Particle* pi = m_particleBuffer.get(i);

        float sum = 0.f;
        m_neighborTable.point_prepare(i);

        int gridCell[8];
        m_gridContainer.findCells(pi->pos, m_smoothRadius/m_unitScale, gridCell);

        for(int cell=0; cell < 8; cell++)
        {
            if(gridCell[cell] == -1) continue;

            int pndx = m_gridContainer.getGridData(gridCell[cell]);

            bool isNeighborTableFull = false;

            while(pndx != -1)
            {
                Particle* pj = m_particleBuffer.get(pndx);
                if(pj == pi)
                {
                    sum += std::pow(h2, 3.f);  //self
                }
                else
                {
                    glm::vec3 pi_pj = (pi->pos - pj->pos) * m_unitScale;
                    float pi_pj_len = glm::length(pi_pj);
                    float r2 = pi_pj_len * pi_pj_len;
                    if (h2 > r2)
                    {
                        float h2_r2 =  h2 - r2;
                        sum += std::pow(h2_r2, 3.f);  //(h^2-r^2)^3

                        if(!m_neighborTable.point_add_neighbor(pndx, std::sqrt(r2)))
                        {
                            isNeighborTableFull = true;
                            break;
                        }
                    }
                }
                pndx = pj->next;
            }

            if (isNeighborTableFull)
            {
                break;
            }

        }

        m_neighborTable.point_commit();

        //m_kernelPoly6 = 315.0f/(64.0f * 3.141592f * h^9);
        pi->density = m_kernelPoly6 * m_particleMass * sum;

        //Calculate the pressure of single particle with the Ideal Gas State Equation
        pi->pressure = (pi->density - m_restDensity) * m_gasConstantK;
    }
}

void SPHSystem::_computeForce()
{
    float h2 = m_smoothRadius * m_smoothRadius;

    for(unsigned int i=0; i<m_particleBuffer.size(); i++)
    {
        Particle* pi = m_particleBuffer.get(i);

        glm::vec3 accel_sum(0,0,0);

        int neighborCounts = m_neighborTable.getNeighborCounts(i);

        for(int j=0; j <neighborCounts; j++)
        {
            unsigned short neighborIndex;
            float r;
            m_neighborTable.getNeighborInfo(i, j, neighborIndex, r);

            Particle* pj = m_particleBuffer.get(neighborIndex);

            //r(i)-r(j)
            glm::vec3 ri_rj = (pi->pos - pj->pos)*m_unitScale;
            //h-r
            float h_r = m_smoothRadius - r;
            //h^2-r^2
            float h2_r2 = h2 - r*r;

            //F_Pressure
            //m_kernelSpiky = -45.0f/(3.141592f * h^6);
            float pterm = -m_particleMass*m_kernelSpiky*h_r*h_r*(pi->pressure+pj->pressure)/(2.f * pi->density * pj->density);
            accel_sum += ri_rj*pterm/r;

            //F_Viscosity
            //m_kernelViscosity = 45.0f/(3.141592f * h^6);
            float vterm = m_kernelViscosity * m_viscosity * h_r * m_particleMass/(pi->density * pj->density);
            accel_sum += (pj->velocity - pi->velocity)*vterm;
        }

        pi->acceleration = accel_sum;
    }
}

void SPHSystem::_advance() {


    float SL2 = m_speedLimiting*m_speedLimiting;

    for(unsigned int i=0; i<m_particleBuffer.size(); i++)
    {
        Particle* p = m_particleBuffer.get(i);

        // Compute Acceleration
        glm::vec3 accel = p->acceleration;

        // Velocity limiting
        float accel_len = glm::length(accel);
        float accel_2 = accel_len * accel_len;

        if(accel_2 > SL2)
        {
            accel *= m_speedLimiting/sqrt(accel_2);
        }

        // Boundary Conditions

        // Z-axis walls
        float diff = 2 * m_unitScale - (p->pos.z - m_sphWallBox.min.z) * m_unitScale;
        if (diff > 0.f )
        {
            glm::vec3 norm(0, 0, 1);
            float adj = m_boundaryStiffness * diff - m_boundaryDampening * glm::dot( norm, p->velocity );
            accel.x += adj * norm.x;
            accel.y += adj * norm.y;
            accel.z += adj * norm.z;
        }

        diff = 2 * m_unitScale - (m_sphWallBox.max.z - p->pos.z)*m_unitScale;
        if (diff > 0.f)
        {
            glm::vec3 norm( 0, 0, -1);
            float adj = m_boundaryStiffness * diff - m_boundaryDampening * glm::dot( norm, p->velocity );
            accel.x += adj * norm.x;
            accel.y += adj * norm.y;
            accel.z += adj * norm.z;
        }

        // X-axis walls
        diff = 2 * m_unitScale - (p->pos.x - m_sphWallBox.min.x)*m_unitScale;
        if (diff > 0.f )
        {
            glm::vec3 norm(1, 0, 0);
            float adj = m_boundaryStiffness * diff - m_boundaryDampening * glm::dot( norm, p->velocity ) ;
            accel.x += adj * norm.x;
            accel.y += adj * norm.y;
            accel.z += adj * norm.z;
        }

        diff = 2 * m_unitScale - (m_sphWallBox.max.x - p->pos.x)*m_unitScale;
        if (diff > 0.f)
        {
            glm::vec3 norm(-1, 0, 0);
            float adj = m_boundaryStiffness * diff - m_boundaryDampening * glm::dot( norm, p->velocity );
            accel.x += adj * norm.x;
            accel.y += adj * norm.y;
            accel.z += adj * norm.z;
        }

        // Y-axis walls
        diff = 2 * m_unitScale - ( p->pos.y - m_sphWallBox.min.y )*m_unitScale;
        if (diff > 0.f)
        {
            glm::vec3 norm(0, 1, 0);
            float adj = m_boundaryStiffness * diff - m_boundaryDampening * glm::dot( norm, p->velocity );
            accel.x += adj * norm.x;
            accel.y += adj * norm.y;
            accel.z += adj * norm.z;
        }
        diff = 2 * m_unitScale - ( m_sphWallBox.max.y - p->pos.y )*m_unitScale;
        if (diff > 0.f)
        {
            glm::vec3 norm(0, -1, 0);
            float adj = m_boundaryStiffness * diff - m_boundaryDampening * glm::dot( norm, p->velocity );
            accel.x += adj * norm.x;
            accel.y += adj * norm.y;
            accel.z += adj * norm.z;
        }

        // Plane gravity
        accel += m_gravityDir;

        p->acceleration = accel;

        m_timeIntegrator->update(p);


    }
}

void SPHSystem::addParticles(const ParticleBox3 &fluidBox, float spacing)
{
    for (float z=fluidBox.max.z; z>=fluidBox.min.z; z-=spacing)
    {
        for (float y=fluidBox.min.y; y<=fluidBox.max.y; y+=spacing)
        {
            for (float x=fluidBox.min.x; x<=fluidBox.max.x; x+=spacing)
            {
                Particle* p = m_particleBuffer.AddParticle();

                p->pos.x = x;
                p->pos.y = y;
                p->pos.z = z;
            }
        }
    }
}
