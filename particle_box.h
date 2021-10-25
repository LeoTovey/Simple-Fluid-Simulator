//
// Created by Leo on 2021/10/24.
//

#ifndef SIMPLE_FLUID_SIMULATOR_PARTICLE_BOX_H
#define SIMPLE_FLUID_SIMULATOR_PARTICLE_BOX_H

#include <vector>
#include "particle.h"

class ParticleBox3
{
public:
    ParticleBox3() {}
    ParticleBox3(const ParticleBox3& other) : min(other.min), max(other.max) {}
    ParticleBox3(const glm::vec3& _min, const glm::vec3& _max) : min(_min), max(_max) {}
    ~ParticleBox3() {}

public:
    glm::vec3 min, max;
};
class ParticleGridContainer {

public:
    ParticleGridContainer();
    virtual ~ParticleGridContainer();

public:
    // Spatial Subdivision
    void init(const ParticleBox3& box, float sim_scale, float cell_size, float border);
    void insertParticles(ParticleBuffer* particleBuffer);
    void findCells(const glm::vec3 & p, float radius, int* gridCell) const;
    int getGridData(int gridIndex);

    const glm::ivec3 * getGridRes() const { return &m_gridRes; }
    const glm::vec3 * getGridMin() const { return &m_gridMin; }
    const glm::vec3 * getGridMax() const { return &m_gridMax; }
    const glm::vec3 * getGridSize() const { return &m_gridSize; }

    int getGridCellIndex(float px, float py, float pz) const;
private:
    // Spatial Grid
    std::vector<int>	m_gridData;
    glm::vec3 			m_gridMin{};				// volume of grid (may not match domain volume exactly)
    glm::vec3 			m_gridMax{};
    glm::ivec3 			m_gridRes{};				// resolution in each axis
    glm::vec3 			m_gridSize{};				// physical size in each axis
    glm::vec3 			m_gridDelta{};
    float				m_gridCellSize{};
};


class NeighborTable
{
public:
    /** reset neighbor table */
    void reset(unsigned short pointCounts);
    /** prepare a point neighbor data */
    void point_prepare(unsigned short ptIndex);
    /** add neighbor data to current point */
    bool point_add_neighbor(unsigned short ptIndex, float distance);
    /** commit point neighbor data to data buf*/
    void point_commit(void);
    /** get point neighbor counts */
    int getNeighborCounts(unsigned short ptIndex) { return m_pointExtraData[ptIndex].neighborCounts; }
    /** get point neighbor information*/
    void getNeighborInfo(unsigned short ptIndex, int index, unsigned short& neighborIndex, float& neighborDistance);

private:
    enum {MAX_NEIGHBOR_COUNTS=80,};

    union PointExtraData
    {
        struct
        {
            unsigned neighborDataOffset : 24;
            unsigned neighborCounts		: 8;
        };

        unsigned int neighborData;
    };

    PointExtraData* m_pointExtraData;
    unsigned int m_pointCounts;
    unsigned int m_pointCapacity;

    unsigned char* m_neighborDataBuf;	//neighbor data buf
    unsigned int m_dataBufSize;			//in bytes
    unsigned int m_dataBufOffset;		//current neighbor data buf offset

    ////// temp data for current point
    unsigned short m_currPoint;
    int m_currNeighborCounts;
    unsigned short m_currNeighborIndex[MAX_NEIGHBOR_COUNTS];
    float m_currNeighborDistance[MAX_NEIGHBOR_COUNTS];

private:
    void _growDataBuf(unsigned int need_size);

public:
    NeighborTable();
    ~NeighborTable();
};


#endif //SIMPLE_FLUID_SIMULATOR_PARTICLE_BOX_H
