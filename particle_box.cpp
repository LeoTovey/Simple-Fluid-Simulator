//
// Created by Leo on 2021/10/24.
//

#include "particle_box.h"



ParticleGridContainer::ParticleGridContainer() = default;

ParticleGridContainer::~ParticleGridContainer() = default;

int ParticleGridContainer::getGridData(int gridIndex)
{
    if (gridIndex<0 || gridIndex>=(int)m_gridData.size())
    {
        return -1;
    }

    return m_gridData[gridIndex];
}

int ParticleGridContainer::getGridCellIndex(float px, float py, float pz) const
{
    int gx = (int)((px - m_gridMin.x) * m_gridDelta.x);
    int gy = (int)((py - m_gridMin.y) * m_gridDelta.y);
    int gz = (int)((pz - m_gridMin.z) * m_gridDelta.z);
    return (gz * m_gridRes.y + gy) * m_gridRes.x + gx;
}

void ParticleGridContainer::init(const ParticleBox3 &box, float sim_scale, float cell_size, float border)
{
    // Ideal grid cell size (gs) = 2 * smoothing radius = 0.02*2 = 0.04
    // Ideal domain size = k*gs/d = k*0.02*2/0.005 = k*8 = {8, 16, 24, 32, 40, 48, ..}
    //    (k = number of cells, gs = cell size, d = simulation scale)
    float world_cellsize = cell_size / sim_scale;

    m_gridMin = box.min;	m_gridMin -= border;
    m_gridMax = box.max;	m_gridMax += border;

    m_gridSize = m_gridMax;
    m_gridSize -= m_gridMin;
    m_gridCellSize = world_cellsize;

    // Determine grid resolution
    m_gridRes.x = (int)ceil(m_gridSize.x / world_cellsize);
    m_gridRes.y = (int)ceil(m_gridSize.y / world_cellsize);
    m_gridRes.z = (int)ceil(m_gridSize.z / world_cellsize);

    // Adjust grid size to multiple of cell size
    m_gridSize.x = m_gridRes.x * cell_size / sim_scale;
    m_gridSize.y = m_gridRes.y * cell_size / sim_scale;
    m_gridSize.z = m_gridRes.z * cell_size / sim_scale;

    // delta = translate from world space to cell #
    m_gridDelta = m_gridRes;
    m_gridDelta /= m_gridSize;

    int gridTotal = (int)(m_gridRes.x * m_gridRes.y * m_gridRes.z);
    m_gridData.resize(gridTotal);
}

void ParticleGridContainer::insertParticles(ParticleBuffer *particleBuffer)
{
    std::fill(m_gridData.begin(), m_gridData.end(), -1);

    Particle* p = particleBuffer->get(0);
    for(unsigned int n=0; n < particleBuffer->size(); n++, p++)
    {
        int gs = getGridCellIndex(p->pos.x, p->pos.y, p->pos.z);
        if ( gs >= 0 && gs < (int)m_gridData.size() )
        {
            p->next = m_gridData[gs];
            m_gridData[gs] =(int) n;
        }
        else p->next = -1;
    }
}

void ParticleGridContainer::findCells(const glm::vec3 &p, float radius, int *gridCell) const
{
    for(int i=0; i<8; i++) gridCell[i]=-1;

    // Compute sphere range
    int sph_min_x = (int)((-radius + p.x - m_gridMin.x) * m_gridDelta.x);
    int sph_min_y = (int)((-radius + p.y - m_gridMin.y) * m_gridDelta.y);
    int sph_min_z = (int)((-radius + p.z - m_gridMin.z) * m_gridDelta.z);

    if ( sph_min_x < 0 ) sph_min_x = 0;
    if ( sph_min_y < 0 ) sph_min_y = 0;
    if ( sph_min_z < 0 ) sph_min_z = 0;

    gridCell[0] = (sph_min_z * m_gridRes.y + sph_min_y) * m_gridRes.x + sph_min_x;
    gridCell[1] = gridCell[0] + 1;
    gridCell[2] = (int)(gridCell[0] + m_gridRes.x);
    gridCell[3] = gridCell[2] + 1;

    if ( sph_min_z+1 < m_gridRes.z )
    {
        gridCell[4] = (int)(gridCell[0] + m_gridRes.y * m_gridRes.x);
        gridCell[5] = gridCell[4] + 1;
        gridCell[6] = (int)(gridCell[4] + m_gridRes.x);
        gridCell[7] = gridCell[6] + 1;
    }
    if ( sph_min_x+1 >= m_gridRes.x )
    {
        gridCell[1] = -1;		gridCell[3] = -1;
        gridCell[5] = -1;		gridCell[7] = -1;
    }
    if ( sph_min_y+1 >= m_gridRes.y )
    {
        gridCell[2] = -1;		gridCell[3] = -1;
        gridCell[6] = -1;		gridCell[7] = -1;
    }
}

//-----------------------------------------------------------------------------------------------------------------
NeighborTable::NeighborTable()
        : m_pointExtraData(0)
        , m_pointCounts(0)
        , m_pointCapacity(0)
        , m_neighborDataBuf(0)
        , m_dataBufSize(0)
        , m_currNeighborCounts(0)
        , m_currPoint(0)
        , m_dataBufOffset(0)
{
}

//-----------------------------------------------------------------------------------------------------------------
NeighborTable::~NeighborTable()
{
    if(m_pointExtraData) free(m_pointExtraData);
    if(m_neighborDataBuf) free(m_neighborDataBuf);
}

//-----------------------------------------------------------------------------------------------------------------
void NeighborTable::reset(unsigned short pointCounts)
{
    int a = sizeof(PointExtraData);
    if(pointCounts>m_pointCapacity)
    {
        if(m_pointExtraData)
        {
            free(m_pointExtraData);
        }
        m_pointExtraData = (PointExtraData*)malloc(sizeof(PointExtraData)*pointCounts);
        m_pointCapacity = pointCounts;
    }

    m_pointCounts = pointCounts;
    memset(m_pointExtraData, 0, sizeof(PointExtraData)*m_pointCapacity);
    m_dataBufOffset = 0;
}

//-----------------------------------------------------------------------------------------------------------------
void NeighborTable::point_prepare(unsigned short ptIndex)
{
    m_currPoint = ptIndex;
    m_currNeighborCounts = 0;
}

//-----------------------------------------------------------------------------------------------------------------
bool NeighborTable::point_add_neighbor(unsigned short ptIndex, float distance)
{
    if (m_currNeighborCounts >= MAX_NEIGHBOR_COUNTS) return false;

    m_currNeighborIndex[m_currNeighborCounts]=ptIndex;
    m_currNeighborDistance[m_currNeighborCounts]=distance;

    m_currNeighborCounts++;
    return true;
}

//-----------------------------------------------------------------------------------------------------------------
void NeighborTable::point_commit(void)
{
    if(m_currNeighborCounts==0) return;

    unsigned int index_size = m_currNeighborCounts*sizeof(unsigned short);
    unsigned int distance_size = m_currNeighborCounts*sizeof(float);

    //grow buf
    if(m_dataBufOffset+index_size+distance_size>m_dataBufSize)
    {
        _growDataBuf(m_dataBufOffset+index_size+distance_size);
    }

    //set neightbor data
    m_pointExtraData[m_currPoint].neighborCounts = m_currNeighborCounts;
    m_pointExtraData[m_currPoint].neighborDataOffset = m_dataBufOffset;

    //copy index data
    memcpy(m_neighborDataBuf+m_dataBufOffset, m_currNeighborIndex, index_size);
    m_dataBufOffset += index_size;

    //copy distance data
    memcpy(m_neighborDataBuf+m_dataBufOffset, m_currNeighborDistance, distance_size);
    m_dataBufOffset += distance_size;
}

//-----------------------------------------------------------------------------------------------------------------
void NeighborTable::_growDataBuf(unsigned int need_size)
{
    unsigned int newSize = m_dataBufSize>0 ? m_dataBufSize : 1;
    while(newSize<need_size) newSize*=2;
    if(newSize<1024)newSize=1024;

    unsigned char* newBuf = (unsigned char*)malloc(newSize);
    if(m_neighborDataBuf)
    {
        memcpy(newBuf, m_neighborDataBuf, m_dataBufSize);
        free(m_neighborDataBuf);
    }
    m_neighborDataBuf = newBuf;
    m_dataBufSize = newSize;
}

//-----------------------------------------------------------------------------------------------------------------
void NeighborTable::getNeighborInfo(unsigned short ptIndex, int index, unsigned short& neighborIndex, float& neighborDistance)
{
    PointExtraData neighData = m_pointExtraData[ptIndex];

    unsigned short* indexBuf = (unsigned short*)(m_neighborDataBuf+neighData.neighborDataOffset);
    float* distanceBuf = (float*)(m_neighborDataBuf + neighData.neighborDataOffset+sizeof(unsigned short)*neighData.neighborCounts);

    neighborIndex = indexBuf[index];
    neighborDistance = distanceBuf[index];
}