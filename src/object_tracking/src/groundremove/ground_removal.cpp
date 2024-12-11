#include "gaus_blur.h"

float rMin = 3.4;
float rMax = 120;

float tHmin = -2.0;
float tHmax = -0.4;

float tHDiff = 0.4;

float hSeonsor = 2.0;

Cell::Cell()
{
    minZ = 1000;
    isGround = false;
}

Cell::~Cell()
{
}

void Cell::updateMinZ(float z) {if(z < minZ) minZ = z;}
void Cell::updateHeight(float h) {height = h;}
void Cell::updateSmoothed(float s) {smoothed = s;}
void Cell::updateHDiff(float hd) {hDiff = hd;}
void Cell::updateGround() {isGround = true; hGround = height;}

bool Cell::isThisGround() {return isGround;}

float Cell::getMinZ() {return minZ;}
float Cell::getHeight() {return height;}
float Cell::getHDiff() {return hDiff;}
float Cell::getSmoothed() {return smoothed;}
float Cell::getHGround() {return hGround;}


//过滤车周围半径 使用欧式距离
void filterCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &filteredCloud)
{
    for(auto i = 0; i < cloud->points.size(); ++i){
        float x = cloud->points[i].x;
        float y = cloud->points[i].y;
        float z = cloud->points[i].z;

        float distance = sqrt(x * x + y * y);
        if(distance <= rMin || distance >= rMax){
            continue;
        }
        else{
            pcl::PointXYZ point;
            point.x = x;
            point.y = y;
            point.z = z;
            filteredCloud->push_back(point);
        }
    }
}

//x y 坐标转化为chP binP坐标
void getCellIndexFromPoints(float x, float y, int &chI, int &binI)
{
    float distance = sqrt(x * x + y * y);
    float chP = (atan2(y, x) + M_PI) / (2 * M_PI);   //范围（0, 1）
    float binP = (distance - rMin) / (rMax - rMin);  //范围 (0, 1)

    chI = floor(chP * numChannel);
    binI = floor(binP * numBin);                                                                                                      
}

//创造映射极坐标网络
void createAndMapPolarGrid(pcl::PointCloud<pcl::PointXYZ>::Ptr  &cloud, 
                           std::array<std::array<Cell, numBin>, numChannel>& polarData)
{
    for(auto i = 0; i < cloud->points.size(); ++i){
        float x = cloud->points[i].x;
        float y = cloud->points[i].y;
        float z = cloud->points[i].z;
    
        int chI, binI;  //角度和半径
        getCellIndexFromPoints(x, y, chI, binI);
        if(chI < 0 || chI >= numChannel || binI < 0 || binI >= numBin)
            continue;
        polarData[chI][binI].updateMinZ(z);            
    }
}

//update HDiff with larger value
void computeHDiffAdjacentCell(std::array<Cell, numBin>& channelData)
{
    for(auto i = 0; i < channelData.size(); ++i)
    {
        if(i == 0){
            float hD = channelData[i].getHeight() - channelData[i+1].getHeight();
            channelData[i].updateHDiff(hD);
        }
        else if(i = channelData.size() - 1){
            float hD = channelData[i].getHeight() - channelData[i-1].getHeight();
            channelData[i].updateHDiff(hD);
        }
        else{
            float prehD = channelData[i].getHeight() - channelData[i-1].getHeight();
            float posthD = channelData[i].getHeight() - channelData[i+1].getHeight();
            if(prehD > posthD)
                channelData[i].updateHDiff(prehD);
            else
                channelData[i].updateHDiff(posthD);
        }
    }
}

//中值滤波处理缺失的地面信息（由于遮挡而常见） 缺失单元的u高度值将替换为相邻单元的中值
void applyMedianFilter(std::array<std::array<Cell, numBin>, numChannel> &polardata)
{
    for(auto channel = 1; channel < polardata.size() - 1; ++channel)
    {
        for(auto bin = 1; bin < polardata[0].size() - 1; ++bin)
        {
            if(!polardata[channel][bin].isThisGround()){
                if(polardata[channel][bin+1].isThisGround() && polardata[channel][bin-1].isThisGround() &&
                   polardata[channel+1][bin].isThisGround() && polardata[channel-1][bin].isThisGround()){
                    std::vector<float> sur{polardata[channel][bin+1].getHeight(), polardata[channel][bin-1].getHeight(),
                                           polardata[channel+1][bin].getHeight(), polardata[channel-1][bin].getHeight()};
                    std::sort(sur.begin(), sur.end());
                    float median = (sur[1] + sur[2]) / 2;
                    polardata[channel][bin].updateHeight(median);  //高度取中值
                    polardata[channel][bin].updateGround();        //是地面
                }
            }
        }
    }
}

void outlierFilter(std::array<std::array<Cell, numBin>, numChannel> &polardata)
{
    for(auto channel = 1; channel < polardata.size() - 1; ++channel){
        for(auto bin = 1; bin < polardata.size() - 2; ++bin){
            if(polardata[channel][bin+1].isThisGround() && polardata[channel][bin+2].isThisGround() &&
               polardata[channel][bin-1].isThisGround() && polardata[channel][bin].isThisGround()){
                float height1 = polardata[channel][bin-1].getHeight();
                float height2 = polardata[channel][bin].getHeight();
                float height3 = polardata[channel][bin+1].getHeight();
                float height4 = polardata[channel][bin+2].getHeight();
                if(height1 != tHmin && height2 == tHmin && height3 != tHmin){
                    float newH = (height1 + height3) / 2;
                    polardata[channel][bin].updateHeight(newH);
                    polardata[channel][bin].updateGround();
                }
                else if(height1 != tHmin && height2 == tHmin && height3 == tHmin && height4 != tHmin){
                    float newH = (height1 + height4) / 2;
                    polardata[channel][bin].updateHeight(newH);
                    polardata[channel][bin].updateGround();
                }
            }
        }
    }
}

void groundRemove(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr nongroundCloud, 
                  pcl::PointCloud<pcl::PointXYZ>::Ptr groundCloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>());

    std::cout << "Ground remove starts!" << std::endl;
    filterCloud(cloud, filteredCloud);
    std::array<std::array<Cell, numBin>, numChannel> polarData;
    createAndMapPolarGrid(filteredCloud, polarData);

    for(auto channel = 0; channel < polarData.size(); ++channel){
        for(auto bin = 0; bin < polarData[0].size(); ++bin){
            float zi = polarData[channel][bin].getMinZ();
            if(zi > tHmin && zi < tHmax){
                polarData[channel][bin].updateHeight(zi);  // 每个Cell栅格都有一个updateHeight
            }
            else if(zi >tHmax){
                polarData[channel][bin].updateHeight(hSeonsor);
            }
            else{
                polarData[channel][bin].updateHeight(tHmin);
            }
        }
        gaussSmoothen(polarData[channel], 1, 3);
        computeHDiffAdjacentCell(polarData[channel]);

        for (auto bin = 0; bin < polarData[0].size(); ++bin){
            if(polarData[channel][bin].getSmoothed() < tHmax && polarData[channel][bin].getHDiff() < tHDiff){  
                polarData[channel][bin].updateGround();  
            }
            else if(polarData[channel][bin].getHeight() < tHmax && polarData[channel][bin].getHDiff() < tHDiff){
                polarData[channel][bin].updateGround();
            }
        }
    }

    applyMedianFilter(polarData);
    outlierFilter(polarData);

    for(auto i = 0; i < filteredCloud->size(); ++i){
        float x = filteredCloud->points[i].x;
        float y = filteredCloud->points[i].y;
        float z = filteredCloud->points[i].z;

        pcl::PointXYZ point;
        point.x = x;
        point.y = y;
        point.z = z;

        int chI, binI;
        getCellIndexFromPoints(x, y, chI, binI);
        if(chI < 0 || chI >= numChannel || binI < 0 || binI >= numBin){
            continue;
        }
        if(polarData[chI][binI].isThisGround()){
            float hGround = polarData[chI][binI].getHGround();
            if(z < (hGround + 0.25)){
                groundCloud->push_back(point);
            }
        }
        else{
            nongroundCloud->push_back(point);
        }
    }
    std::cout << "初始点云： " << cloud->size() << "高点： " << nongroundCloud->size() << " 地面点： " << groundCloud->size() << std::endl;
}



