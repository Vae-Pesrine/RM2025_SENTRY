#ifndef GROUND_REMOVAL_H
#define GROUND_REMOVAL_H

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

const int numChannel = 80;
const int numBin = 120;

extern float rMin;
extern float rMax;

extern float tHmin;
extern float tHmax;

extern float tHDiff;

extern float hSeonsor;

class Cell
{
private:
    float smoothed;
    float height;
    float hDiff;
    float hGround;
    float minZ;
    bool isGround;

public:
    Cell();
    ~Cell();

    void updateMinZ(float z); 
    void updateHeight(float h); 
    void updateSmoothed(float s); 
    void updateHDiff(float hd); 
    void updateGround(); 
    
    bool isThisGround(); 
    
    float getMinZ(); 
    float getHeight(); 
    float getHDiff(); 
    float getSmoothed(); 
    float getHGround(); 
};

void getCellIndexFromPoints(float x, float y, int &chI, int &binI);

void createAndMapPolarGrid(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, std::array<std::array<Cell, numBin>, numChannel>& polarData);

void computeHDiffAdjacentCell(std::array<Cell, numBin>& channelData);

void applyMedianFilter(std::array<std::array<Cell, numBin>, numChannel> &polardata);

void outlierFilter(std::array<std::array<Cell, numBin>, numChannel> &polardata);


void groundRemove(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr elevatedCloud, 
                  pcl::PointCloud<pcl::PointXYZ>::Ptr groundcloud);

#endif // GROUND_REMOVAL_H