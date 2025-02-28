#include <math.h>
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
 
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
 
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32.h>
 
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
 
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
 
//用于地形分析的 ROS 节点，它使用来自激光雷达的点云数据来估算地形，并对地形点云进行处理和分析
using namespace std;
 
const double PI = 3.1415926;
 
double scanVoxelSize = 0.05;//点云下采样时体素的大小，以米为单位
double decayTime = 2.0;//在点云中保留旧点的时间（s），超过这个时间的点可能会从处理中移除
double noDecayDis = 4.0;//车辆点云检测有效距离
double clearingDis = 8.0;//在该距离的点将被清除，可能用于避开障碍物或清理环境
bool clearingCloud = false;//指示是否应该清理点云
bool useSorting = true;//是否使用排序方法来处理点云，可能用于地面估计或障碍物检测
double quantileZ = 0.25;//定义了要作为地面考虑的高度阈值（取体素中前quantileZ比例分数的高度点云）
bool considerDrop = false;//是否考虑高度差异（如坑洼或下坡）作为障碍物
bool limitGroundLift = false;//是否限制地面升降的最大值
double maxGroundLift = 0.15;//允许的最大地面升降高度。
bool clearDyObs = false;//是否重置或清除动态障碍物
double minDyObsDis = 0.3;//动态障碍物检测的最小距离
double minDyObsAngle = 0;//动态障碍物检测的最小角度
double minDyObsRelZ = -0.5;//水平面相对于车体的高度
double absDyObsRelZThre = 0.2;//动态障碍物的绝对 Z 坐标阈值
double minDyObsVFOV = -16.0;//动态障碍物垂直视场范围的最小和最大值（/左侧最大转向角？）
double maxDyObsVFOV = 16.0;//右侧最大转向角？
int minDyObsPointNum = 1;//动态障碍物检测所需的最小点数
bool noDataObstacle = false;//是否将无数据区域视为障碍物
int noDataBlockSkipNum = 0;//在将无数据区域视为障碍物之前要重复评估几次（遍历）
int minBlockPointNum = 10;// 区块内最小的点数，决定了该区块是否包含足够的数据
double vehicleHeight = 1.5;//车辆的高度
int voxelPointUpdateThre = 100;//体素更新的点数阈值和时间阈值。（同一个位置的雷达点数阈值）
double voxelTimeUpdateThre = 2.0;
double minRelZ = -1.5;// 未使用地面分割时，裁剪点云时的最小高度
double maxRelZ = 0.2;// 未使用地面分割时，裁剪点云时的最大高度
double disRatioZ = 0.2;// 点云处理的高度与距离的比例-与激光雷达性能相关
 
// terrain voxel parameters
//地形体素
float terrainVoxelSize = 1.0;//体素的边长大小（米）
int terrainVoxelShiftX = 0;//地面体素网格X位置（map坐标系下，体素个数计量）
int terrainVoxelShiftY = 0;
const int terrainVoxelWidth = 21;//体素网格在X和Y方向上的宽度，即网格中有多少个体素
int terrainVoxelHalfWidth = (terrainVoxelWidth - 1) / 2;
const int terrainVoxelNum = terrainVoxelWidth * terrainVoxelWidth;//体素网格数量
 
// planar voxel parameters
//地面体素
float planarVoxelSize = 0.2;//定义了平面体素的边长（0.2m）
const int planarVoxelWidth = 51;//车体周围5个terrainVoxel范围，（1*（2*5）/0.2）+1定义了平面体素网格在X和Y方向上的宽度（体素网格数量）
int planarVoxelHalfWidth = (planarVoxelWidth - 1) / 2;
const int planarVoxelNum = planarVoxelWidth * planarVoxelWidth;
 
pcl::PointCloud<pcl::PointXYZI>::Ptr
    laserCloud(new pcl::PointCloud<pcl::PointXYZI>());//初始化一个新的点云对象 ，用于存储原始激光雷达数据
pcl::PointCloud<pcl::PointXYZI>::Ptr
    laserCloudCrop(new pcl::PointCloud<pcl::PointXYZI>());//用于存储裁剪后的点云数据
pcl::PointCloud<pcl::PointXYZI>::Ptr
    laserCloudDwz(new pcl::PointCloud<pcl::PointXYZI>());//用于存储下采样后的点云。
pcl::PointCloud<pcl::PointXYZI>::Ptr
    terrainCloud(new pcl::PointCloud<pcl::PointXYZI>());//用于存储与地形相关的点云数据（车辆周围一定距离的点云）
pcl::PointCloud<pcl::PointXYZI>::Ptr
    terrainCloudElev(new pcl::PointCloud<pcl::PointXYZI>());//该点云数组存储障碍物点云的信息(x,y,z)，最后发布到map数组中
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloud[terrainVoxelNum];//数组中每个元素都是一个指向 pcl::PointCloud 的指针，用于存储体素化后的障碍物点云数据。
 
int terrainVoxelUpdateNum[terrainVoxelNum] = {0};//用于存储每个地形体素被更新的次数，元素数量=体素数量
float terrainVoxelUpdateTime[terrainVoxelNum] = {0};//地形体素的更新时间
float planarVoxelElev[planarVoxelNum] = {0};//保存了体素id附近点云高程的最小值（视为地面高度）
int planarVoxelEdge[planarVoxelNum] = {0};//用于标记每个平面体素是否位于某种边缘（数据缺失/障碍）
int planarVoxelDyObs[planarVoxelNum] = {0};//记录每个平面体素中动态障碍物的数量或状态
vector<float> planarPointElev[planarVoxelNum];//用于存储第i个平面体素（planarVoxel）内所有符合要求高度的点的高度数据
 
double laserCloudTime = 0;
bool newlaserCloud = false;
 
double systemInitTime = 0;
bool systemInited = false;
int noDataInited = 0;//数据初始化状态noDataInited=0，说明这是第一次接收到数据
 
float vehicleRoll = 0, vehiclePitch = 0, vehicleYaw = 0;
float vehicleX = 0, vehicleY = 0, vehicleZ = 0;//cle当前位置
float vehicleXRec = 0, vehicleYRec = 0;//rec初始位置
 
float sinVehicleRoll = 0, cosVehicleRoll = 0;
float sinVehiclePitch = 0, cosVehiclePitch = 0;
float sinVehicleYaw = 0, cosVehicleYaw = 0;
 
pcl::VoxelGrid<pcl::PointXYZI> downSizeFilter;
 
// state estimation callback function
//处理由 ROS 话题 /state_estimation 发布的里程计信息（车辆）
void odometryHandler(const nav_msgs::Odometry::ConstPtr &odom) {
  double roll, pitch, yaw;
  geometry_msgs::Quaternion geoQuat = odom->pose.pose.orientation;//四元数变量
  tf::Matrix3x3(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w))//使用 tf 库（转换库）将四元数转换为欧拉角
      .getRPY(roll, pitch, yaw);//提取出绕X轴的翻滚角（roll），绕Y轴的俯仰角（pitch），和绕Z轴的偏航角
 
  vehicleRoll = roll;
  vehiclePitch = pitch;
  vehicleYaw = yaw;
  vehicleX = odom->pose.pose.position.x;//获得车辆里程计坐标（map）
  vehicleY = odom->pose.pose.position.y;
  vehicleZ = odom->pose.pose.position.z;
 
  //提前计算并存储这些常用的三角函数值
  sinVehicleRoll = sin(vehicleRoll);
  cosVehicleRoll = cos(vehicleRoll);
  sinVehiclePitch = sin(vehiclePitch);
  cosVehiclePitch = cos(vehiclePitch);
  sinVehicleYaw = sin(vehicleYaw);
  cosVehicleYaw = cos(vehicleYaw);
 
  if (noDataInited == 0) {//如果 noDataInited 标志为0，说明这是新接收数据
    vehicleXRec = vehicleX;//记录下收到坐标时的车辆位置（车辆初始位置）
    vehicleYRec = vehicleY;
    noDataInited = 1;
  }
  if (noDataInited == 1) {
    float dis = sqrt((vehicleX - vehicleXRec) * (vehicleX - vehicleXRec) +
                     (vehicleY - vehicleYRec) * (vehicleY - vehicleYRec));//计算当前位置与初始位置的距离
    if (dis >= noDecayDis)//车辆移动超过一定距离（车辆移动距离超过了一次里程计雷达检测的距离，造成点云丢失）
      noDataInited = 2;//将noDataInited状态更新=2
  }
}
 
// registered laser scan callback function
//当接收到激光雷达扫描数据时被调用的
void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloud2) {
  laserCloudTime = laserCloud2->header.stamp.toSec();//将点云消息的时间戳转换为秒
 
  if (!systemInited) {//如果系统未初始化
    systemInitTime = laserCloudTime;//记录初始化时间为点云时间
    systemInited = true;
  }
 
  laserCloud->clear();//清空当前的laserCloud 点云，准备存储新的点云数据）
  pcl::fromROSMsg(*laserCloud2, *laserCloud);//将 ROS中laserCloud2点云消息转换为 PCL点云格式的laserCloud
 
  pcl::PointXYZI point;
  laserCloudCrop->clear();//清空用于存储裁剪后点云的容器 laserCloudCrop
  int laserCloudSize = laserCloud->points.size();//遍历点云
  for (int i = 0; i < laserCloudSize; i++) {
    point = laserCloud->points[i];
 
    float pointX = point.x;//点云暂存
    float pointY = point.y;
    float pointZ = point.z;
 
    float dis = sqrt((pointX - vehicleX) * (pointX - vehicleX) +
                     (pointY - vehicleY) * (pointY - vehicleY));//计算点云中的点和当前车辆的距离
    //pointZ减去车辆的Z坐标是否大于一个计算出的最小相对高度 (minRelZ) 减去一个基于距离比例 (disRatioZ) 乘以点到车辆的水平距离 (dis) 的值，确保点不在车辆下方过于接近地面的位置。
    //在近距离时维持精确的地面追踪，而在远距离时允许更大的地形高度变化。
    if (pointZ - vehicleZ > minRelZ - disRatioZ * dis &&
        pointZ - vehicleZ < maxRelZ + disRatioZ * dis &&
        dis < terrainVoxelSize * (terrainVoxelHalfWidth + 1)) {//点与车辆的水平距离 (dis) 是否小于地形体素的尺寸 (terrainVoxelSize) 乘以体素的半宽度（按数量）加一
      point.x = pointX;//筛选后的点云
      point.y = pointY;
      point.z = pointZ;
      point.intensity = laserCloudTime - systemInitTime;//点云强度值设置为点云更新时间（当前时间-系统时间）
      laserCloudCrop->push_back(point);
    }
  }
 
  newlaserCloud = true;//更新收到点云状态
}
 
// joystick callback function
void joystickHandler(const sensor_msgs::Joy::ConstPtr &joy) {
  if (joy->buttons[5] > 0.5) {
    noDataInited = 0;
    clearingCloud = true;
  }
}
 
// cloud clearing callback function
void clearingHandler(const std_msgs::Float32::ConstPtr &dis) {
  noDataInited = 0;
  clearingDis = dis->data;
  clearingCloud = true;
}
 
int main(int argc, char **argv) {
  ros::init(argc, argv, "terrainAnalysis");
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate = ros::NodeHandle("~");
 
  nhPrivate.getParam("scanVoxelSize", scanVoxelSize);
  nhPrivate.getParam("decayTime", decayTime);
  nhPrivate.getParam("noDecayDis", noDecayDis);
  nhPrivate.getParam("clearingDis", clearingDis);
  nhPrivate.getParam("useSorting", useSorting);
  nhPrivate.getParam("quantileZ", quantileZ);
  nhPrivate.getParam("considerDrop", considerDrop);
  nhPrivate.getParam("limitGroundLift", limitGroundLift);
  nhPrivate.getParam("maxGroundLift", maxGroundLift);
  nhPrivate.getParam("clearDyObs", clearDyObs);
  nhPrivate.getParam("minDyObsDis", minDyObsDis);
  nhPrivate.getParam("minDyObsAngle", minDyObsAngle);
  nhPrivate.getParam("minDyObsRelZ", minDyObsRelZ);
  nhPrivate.getParam("absDyObsRelZThre", absDyObsRelZThre);
  nhPrivate.getParam("minDyObsVFOV", minDyObsVFOV);
  nhPrivate.getParam("maxDyObsVFOV", maxDyObsVFOV);
  nhPrivate.getParam("minDyObsPointNum", minDyObsPointNum);
  nhPrivate.getParam("noDataObstacle", noDataObstacle);
  nhPrivate.getParam("noDataBlockSkipNum", noDataBlockSkipNum);
  nhPrivate.getParam("minBlockPointNum", minBlockPointNum);
  nhPrivate.getParam("vehicleHeight", vehicleHeight);
  nhPrivate.getParam("voxelPointUpdateThre", voxelPointUpdateThre);
  nhPrivate.getParam("voxelTimeUpdateThre", voxelTimeUpdateThre);
  nhPrivate.getParam("minRelZ", minRelZ);
  nhPrivate.getParam("maxRelZ", maxRelZ);
  nhPrivate.getParam("disRatioZ", disRatioZ);
 
  ros::Subscriber subOdometry =
      nh.subscribe<nav_msgs::Odometry>("/state_estimation", 5, odometryHandler);
 
  ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(
      "/registered_scan", 5, laserCloudHandler);
 
  ros::Subscriber subJoystick =
      nh.subscribe<sensor_msgs::Joy>("/joy", 5, joystickHandler);
 
  ros::Subscriber subClearing =
      nh.subscribe<std_msgs::Float32>("/map_clearing", 5, clearingHandler);
 
  ros::Publisher pubLaserCloud =
      nh.advertise<sensor_msgs::PointCloud2>("/terrain_map", 2);
 
  for (int i = 0; i < terrainVoxelNum; i++) {
    terrainVoxelCloud[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
  }
 
  downSizeFilter.setLeafSize(scanVoxelSize, scanVoxelSize, scanVoxelSize);
 
  ros::Rate rate(100);
  bool status = ros::ok();
  while (status) {
    ros::spinOnce();
 
    if (newlaserCloud) {//如果收到雷达点云，并经过laserCloudHandler回调函数降采样后
      newlaserCloud = false;
 
      // terrain voxel roll over
      //代码块1.更新车辆周围的地形体素（voxel）网格，体素网格以车辆为中心
      float terrainVoxelCenX = terrainVoxelSize * terrainVoxelShiftX;//体素中心坐标=体素边长*体素x坐标（map坐标系下的体素数量）
      float terrainVoxelCenY = terrainVoxelSize * terrainVoxelShiftY;
 
      //车辆是否已经移动到当前体素网格中心向西（负 X 方向）的一定距离以上
      // 车辆位置X-地面体素中心X < 负的一个体素网格大小
      //车辆在移动时，更新围绕车辆的体素网格以确保网格始终以车辆为中心（体素中心西移动，对应体素列数据向西移动）
      while (vehicleX - terrainVoxelCenX < -terrainVoxelSize) {
        for (int indY = 0; indY < terrainVoxelWidth; indY++) {//indY计数器，当indY小于体素宽度（数量）
          pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr =//terrainVoxelCloudPtr（点云数组），备份（最东（右）侧）terrainVoxelCloud体素网格树组中的点云
              terrainVoxelCloud[terrainVoxelWidth * (terrainVoxelWidth - 1) +//terrainVoxelWidth - 1 确保选择最右侧列（先数列，再数行），而 indY 用于遍历该列的每个体素
                                indY];
          for (int indX = terrainVoxelWidth - 1; indX >= 1; indX--) {//对X遍历（列）
            terrainVoxelCloud[terrainVoxelWidth * indX + indY] =//每列（除了最西侧列）的数据向西移动一个体素（N列的数据移动到第N-1列的位置）
                terrainVoxelCloud[terrainVoxelWidth * (indX - 1) + indY];
          }
          terrainVoxelCloud[indY] = terrainVoxelCloudPtr;//备份的最东侧（最右侧）列的点云指针（terrainVoxelCloudPtr）赋给网格的最西侧列的对应位置（避免了重新分配内存。我们重用了之前的点云对象来代表新的列）
          terrainVoxelCloud[indY]->clear();//清空了最西侧列的点云数据。由于这个点云对象之前代表了网格的最东侧列，现在被移动到了最西侧，因此需要清除其中的旧数据，以便可以存储新的点云数据。
        }
        terrainVoxelShiftX--;//将体素中心向西移动一格（terrainVoxelSize*1大小）
        terrainVoxelCenX = terrainVoxelSize * terrainVoxelShiftX;//更新后（体素滚动后）体素中心坐标
      }
 
      //下述3个while的作用同上
      //车辆位置X-地面体素中心X >一个体素网格大小()
      while (vehicleX - terrainVoxelCenX > terrainVoxelSize) {
        for (int indY = 0; indY < terrainVoxelWidth; indY++) {
          pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr =
              terrainVoxelCloud[indY];
          for (int indX = 0; indX < terrainVoxelWidth - 1; indX++) {
            terrainVoxelCloud[terrainVoxelWidth * indX + indY] =
                terrainVoxelCloud[terrainVoxelWidth * (indX + 1) + indY];
          }
          terrainVoxelCloud[terrainVoxelWidth * (terrainVoxelWidth - 1) +
                            indY] = terrainVoxelCloudPtr;
          terrainVoxelCloud[terrainVoxelWidth * (terrainVoxelWidth - 1) + indY]
              ->clear();
        }
        terrainVoxelShiftX++;
        terrainVoxelCenX = terrainVoxelSize * terrainVoxelShiftX;
      }
 
      // 车辆位置Y-地面体素中心Y < 负的一个体素网格大小
      while (vehicleY - terrainVoxelCenY < -terrainVoxelSize) {
        for (int indX = 0; indX < terrainVoxelWidth; indX++) {
          pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr =
              terrainVoxelCloud[terrainVoxelWidth * indX +
                                (terrainVoxelWidth - 1)];
          for (int indY = terrainVoxelWidth - 1; indY >= 1; indY--) {
            terrainVoxelCloud[terrainVoxelWidth * indX + indY] =
                terrainVoxelCloud[terrainVoxelWidth * indX + (indY - 1)];
          }
          terrainVoxelCloud[terrainVoxelWidth * indX] = terrainVoxelCloudPtr;
          terrainVoxelCloud[terrainVoxelWidth * indX]->clear();
        }
        terrainVoxelShiftY--;
        terrainVoxelCenY = terrainVoxelSize * terrainVoxelShiftY;
      }
 
      // 车辆位置Y-地面体素中心Y >一个体素网格大小
      while (vehicleY - terrainVoxelCenY > terrainVoxelSize) {
        for (int indX = 0; indX < terrainVoxelWidth; indX++) {
          pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr =
              terrainVoxelCloud[terrainVoxelWidth * indX];
          for (int indY = 0; indY < terrainVoxelWidth - 1; indY++) {
            terrainVoxelCloud[terrainVoxelWidth * indX + indY] =
                terrainVoxelCloud[terrainVoxelWidth * indX + (indY + 1)];
          }
          terrainVoxelCloud[terrainVoxelWidth * indX +
                            (terrainVoxelWidth - 1)] = terrainVoxelCloudPtr;
          terrainVoxelCloud[terrainVoxelWidth * indX + (terrainVoxelWidth - 1)]
              ->clear();
        }
        terrainVoxelShiftY++;
        terrainVoxelCenY = terrainVoxelSize * terrainVoxelShiftY;
      }
 
      // stack registered laser scans
      //将map坐标系的下的laserCloudCrop点转换到体素坐标系下（中间经过车体坐标系转换）
      pcl::PointXYZI point;
      int laserCloudCropSize = laserCloudCrop->points.size();
      for (int i = 0; i < laserCloudCropSize; i++) {//对裁减后雷达的点云进行遍历
        point = laserCloudCrop->points[i];
 
        //计算点在体素网格中的位置
        int indX = int((point.x - vehicleX + //当前点(map)相对于车体中心(map)的位置（以车体坐标系为基准）
                          terrainVoxelSize / 2) //取整数时会四舍五入，加上半个体素网格的大小，保证取整后体素坐标>point.x - vehicleX（不满一格的体素坐标按一格算）
                          / terrainVoxelSize) //将长度转化为离散的网格坐标
                          +terrainVoxelHalfWidth;//将计算得到的体素索引（indX）从一个相对于车辆中心的坐标系统，转换到体素网格的索引坐标系（terrainVoxelHalfWidth加了一个体素坐标系的偏移量）
        int indY = int((point.y - vehicleY + terrainVoxelSize / 2) /
                       terrainVoxelSize) +
                   terrainVoxelHalfWidth;
 
        if (point.x - vehicleX + terrainVoxelSize / 2 < 0)//对于车辆西测坐标
          indX--;//在四舍五人时，加了terrainVoxelSize / 2 ，indX--保证体素坐标在point.x - vehicleX左侧（不满一格的体素坐标按一格算）
        if (point.y - vehicleY + terrainVoxelSize / 2 < 0)
          indY--;
 
        if (indX >= 0 && indX < terrainVoxelWidth && indY >= 0 &&//因为有+terrainVoxelHalfWidth坐标系偏移，会将车体的负坐标转为体素正坐标
            indY < terrainVoxelWidth) {
          terrainVoxelCloud[terrainVoxelWidth * indX + indY]->push_back(point);//将对应的point点坐标放入到一维的体素坐标索引数组中
          terrainVoxelUpdateNum[terrainVoxelWidth * indX + indY]++;//该体素中所包含点云的数量+1
        }
      }
 
      //判断是否需要更新体素、执行降采样、筛选点云数据，到最后更新体素状态的整个流程（确保了体素网格中的数据反映了最新的环境状态）
      for (int ind = 0; ind < terrainVoxelNum; ind++) {//体素网格遍历
        //是否需要更新当前体素
        if (terrainVoxelUpdateNum[ind] >= voxelPointUpdateThre ||//该体素网格中点的数量>更新阈值（数据已经足够多，需要进行处理（如降采样））
            laserCloudTime - systemInitTime - terrainVoxelUpdateTime[ind] >=//（laserCloudTime - systemInitTime）当前体素更新时间-terrainVoxelUpdateTime体素ind上一个更新时间
                voxelTimeUpdateThre ||
            clearingCloud) {//收到需要清理点云的命令
          pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr =//terrainVoxelCloudPtr是点云数组的首地址指针（该数组包含一系列点云）
              terrainVoxelCloud[ind];//获取索引为 ind 的体素中的点云数据
 
          laserCloudDwz->clear();//清空降采样点云容器
          downSizeFilter.setInputCloud(terrainVoxelCloudPtr);//将要进行降采样的点云数据（即 terrainVoxelCloudPtr 指向的点云）设置为降采样滤波器 downSizeFilter 的输入
          downSizeFilter.filter(*laserCloudDwz);//对输入体素中的ind位置点云数组terrainVoxelCloudPtr 进行降采样，并将降采样后的点云存储在 laserCloudDwz 中
 
          terrainVoxelCloudPtr->clear();//清空terrainVoxelCloudPtr指针数组
          int laserCloudDwzSize = laserCloudDwz->points.size();
          for (int i = 0; i < laserCloudDwzSize; i++) {//对滤波后第ind体素里的点云数据遍历
            point = laserCloudDwz->points[i];
            float dis = sqrt((point.x - vehicleX) * (point.x - vehicleX) +
                             (point.y - vehicleY) * (point.y - vehicleY));//计算点与车辆距离
            if (point.z - vehicleZ > minRelZ - disRatioZ * dis &&
                point.z - vehicleZ < maxRelZ + disRatioZ * dis &&//点云高度满足要求
                (laserCloudTime - systemInitTime - point.intensity <  //（时间范围）旧点云保留时间<decayTime阈值
                     decayTime ||
                 dis < noDecayDis) &&//点云与车体距离在一定距离范围内
                !(dis < clearingDis && clearingCloud)) {//没有进行清理点云
              terrainVoxelCloudPtr->push_back(point);//将筛选后的点放入terrainVoxelCloudPtr数组
            }
          }
 
          terrainVoxelUpdateNum[ind] = 0;//ind 的体素的更新次数重置为0
          terrainVoxelUpdateTime[ind] = laserCloudTime - systemInitTime;//当前体素的最后更新时间为当前激光雷达扫描数据的时间减去系统初始化时间（以系统初始化时间为基准）
        }
      }
 
      terrainCloud->clear();//清空terrainCloud点云
      for (int indX = terrainVoxelHalfWidth - 5;//遍历特定区域的体素（terrainVoxelHalfWidth在车辆中心），即遍历车辆中心5范围内的体素
           indX <= terrainVoxelHalfWidth + 5; indX++) {
        for (int indY = terrainVoxelHalfWidth - 5;
             indY <= terrainVoxelHalfWidth + 5; indY++) {
          *terrainCloud += *terrainVoxelCloud[terrainVoxelWidth * indX + indY];//将上述体素范围内的点云全部赋值到terrainCloud
        }
      }
 
      // estimate ground and compute elevation for each point
      //为地面估计和每个点的高程计算准备相应的数据结构（初始化）
      for (int i = 0; i < planarVoxelNum; i++) {//循环遍历所有平面体素
        planarVoxelElev[i] = 0;//每个平面体素的高程最小值（planarVoxelElev）初始化为0
        planarVoxelEdge[i] = 0;//将每个体素的边缘标记（planarVoxelEdge）初始化为0
        planarVoxelDyObs[i] = 0;//将每个体素的动态障碍物标记（planarVoxelDyObs）初始化为0
        planarPointElev[i].clear();//清空每个体素中用于存储点的高程值的向量
      }
 
 
 
      //重新在车辆周围（5）划分网格，并将该点高度z赋值给附近的3*3网格，对当前点进行遍历，将符合要求高度和角度的点视为障碍物，并将对应的体素网格障碍物记数+1
      int terrainCloudSize = terrainCloud->points.size();//车辆相邻5个terrainVoxelsize网格范围内的点
      for (int i = 0; i < terrainCloudSize; i++) {
         //1.遍历车辆周围的terrainCloud 中的点云数据，以planarVoxelSize为长度划分网格，将车辆一定高度内的点添加到所在和相邻体素网格向量planarPointElev中
        point = terrainCloud->points[i];
        int indX =
            int((point.x - vehicleX + planarVoxelSize / 2) / planarVoxelSize) +//以planarVoxelSize为长度划分网格，计算每个点在平面体素网格中的位置
            planarVoxelHalfWidth;
        int indY =
            int((point.y - vehicleY + planarVoxelSize / 2) / planarVoxelSize) +
            planarVoxelHalfWidth;
 
        if (point.x - vehicleX + planarVoxelSize / 2 < 0)
          indX--;
        if (point.y - vehicleY + planarVoxelSize / 2 < 0)
          indY--;
        //如果点的在车辆中心高度的-minRelZmaxRelZ内，则将其高度值添加到周围体素（3*3）的 planarPointElev 数组中
        if (point.z - vehicleZ > minRelZ && point.z - vehicleZ < maxRelZ) {
          for (int dX = -1; dX <= 1; dX++) {//遍历点周围的体素（即当前体素和其相邻的八个体素，3*3）
            for (int dY = -1; dY <= 1; dY++) {
              if (indX + dX >= 0 && indX + dX < planarVoxelWidth &&
                  indY + dY >= 0 && indY + dY < planarVoxelWidth) {//如果周围8个体素在体素网格范围内
                planarPointElev[planarVoxelWidth * (indX + dX) + indY + dY]//将点的高度（point.z）添加到其所在或相邻体素的高度数组（planarPointElev）中
                    .push_back(point.z);
              }
            }
          }
        }
 
        //2.计算每个点相对于车辆的位置和角度来判断是否可能是一个动态障碍物
        if (clearDyObs) {//重置障碍物记数
          if (indX >= 0 && indX < planarVoxelWidth && indY >= 0 &&
              indY < planarVoxelWidth) {
            float pointX1 = point.x - vehicleX;//pointX1与车辆的X相对距离
            float pointY1 = point.y - vehicleY;
            float pointZ1 = point.z - vehicleZ;
 
            float dis1 = sqrt(pointX1 * pointX1 + pointY1 * pointY1);//与车辆距离
            if (dis1 > minDyObsDis) {//点与车辆距离大于最小障碍物检测距离
              float angle1 = atan2(pointZ1 - minDyObsRelZ, dis1) * 180.0 / PI;//计算点相对于水平面的角度，pointZ1 - minDyObsRelZ 计算点高度与一个基准高度的差值，dis1 是点到车辆的水平距离
              if (angle1 > minDyObsAngle) {//如果点的角度大于最小动态障碍物检测角度 minDyObsAngle，则认为该点有可能是动态障碍物，继续后续的计算
              //考虑到yaw,pitch,roll角的共同影响，将map坐标系的点经过yaw,再经过pitch，最后经过roll变换到车体坐标系
                float pointX2 =
                    pointX1 * cosVehicleYaw + pointY1 * sinVehicleYaw;//将map坐标系下的点转换到车体坐标系
                float pointY2 =
                    -pointX1 * sinVehicleYaw + pointY1 * cosVehicleYaw;
                float pointZ2 = pointZ1;
 
                float pointX3 =
                    pointX2 * cosVehiclePitch - pointZ2 * sinVehiclePitch;//根据Pitch角坐标转换
                float pointY3 = pointY2;
                float pointZ3 =
                    pointX2 * sinVehiclePitch + pointZ2 * cosVehiclePitch;//应用车辆的俯仰角（vehiclePitch）调整点的 Z 坐标
 
                float pointX4 = pointX3;//最后，应用车辆的横滚角（vehicleRoll）完成坐标转换
                float pointY4 =
                    pointY3 * cosVehicleRoll + pointZ3 * sinVehicleRoll;
                float pointZ4 =
                    -pointY3 * sinVehicleRoll + pointZ3 * cosVehicleRoll;
 
                float dis4 = sqrt(pointX4 * pointX4 + pointY4 * pointY4);//在车体坐标系中，计算相对于车体的距离
                float angle4 = atan2(pointZ4, dis4) * 180.0 / PI;//在车体坐标系中，计算相对于车体的角度
                if (angle4 > minDyObsVFOV && angle4 < maxDyObsVFOV || fabs(pointZ4) < absDyObsRelZThre) {//该点在垂直视角ObsVFOV范围内或者与车辆的高度差pointZ4<阈值，则认为其为障碍物
                  planarVoxelDyObs[planarVoxelWidth * indX + indY]++;//在该体素区域内检测到了可能的动态障碍物，对应体素的动态障碍物计数++
                }
              }
            } else {//当点与车辆的距离较近时。
              planarVoxelDyObs[planarVoxelWidth * indX + indY] +=
                  minDyObsPointNum;//直接将 planarVoxelDyObs 数组对应体素的值增加一个固定的最小点数 (minDyObsPointNum)。这表示即使没有进一步检测该点的角度，也认为该体素区域可能存在动态障碍物
            }
          }
        }
      }
 
 
 
      if (clearDyObs) {//l通过laserCloudCrop对这些点的重新评估，如果它们的角度不符合动态障碍物的特征，则将相关体素的计数重置。
        for (int i = 0; i < laserCloudCropSize; i++) {//对裁减后的雷达点云遍历
          point = laserCloudCrop->points[i];
 
          int indX = int((point.x - vehicleX + planarVoxelSize / 2) /  
                         planarVoxelSize) +
                     planarVoxelHalfWidth;
          int indY = int((point.y - vehicleY + planarVoxelSize / 2) /
                         planarVoxelSize) +
                     planarVoxelHalfWidth;
 
          if (point.x - vehicleX + planarVoxelSize / 2 < 0)
            indX--;
          if (point.y - vehicleY + planarVoxelSize / 2 < 0)
            indY--;
 
          if (indX >= 0 && indX < planarVoxelWidth && indY >= 0 && //车体周围5个terrainVoxel范围，planarVoxelSize=（1*（2*5）/0.2）+1
              indY < planarVoxelWidth) {
            float pointX1 = point.x - vehicleX;
            float pointY1 = point.y - vehicleY;
            float pointZ1 = point.z - vehicleZ;
 
            float dis1 = sqrt(pointX1 * pointX1 + pointY1 * pointY1);//点云与车体距离
            float angle1 = atan2(pointZ1 - minDyObsRelZ, dis1) * 180.0 / PI;//点云与地面高度夹角
            if (angle1 > minDyObsAngle) {
              planarVoxelDyObs[planarVoxelWidth * indX + indY] = 0;//清空障碍物记数
            }
          }
        }
      }
 
      //在地形分析中用于估算每个平面体素（planarVoxel）的高度
      //通过考虑每个体素内点云的分布，可以得到一个更加平滑且真实的地面高度估算
      if (useSorting) {//是否使用排序
        for (int i = 0; i < planarVoxelNum; i++) {//遍历所有平面体素
          int planarPointElevSize = planarPointElev[i].size();
          if (planarPointElevSize > 0) {
            sort(planarPointElev[i].begin(), planarPointElev[i].end());//对该体素内的点云按高度升序（planarPointElev[0]最低）
 
            int quantileID = int(quantileZ * planarPointElevSize);//用于估算地面高度的分位数的索引。quantileZ 是一个预设的比例值，决定取高度值的哪个分位数（前1/4）
            if (quantileID < 0)//确保 quantileID 在合理的范围内
              quantileID = 0;
            else if (quantileID >= planarPointElevSize)
              quantileID = planarPointElevSize - 1;
 
            if (planarPointElev[i][quantileID] >
                    planarPointElev[i][0] + maxGroundLift &&//如果体素中的最低点和分位数点之间的高度差超过 maxGroundLift
                limitGroundLift) {
              planarVoxelElev[i] = planarPointElev[i][0] + maxGroundLift;//则将平面体素最小高度设置为最低点加上 maxGroundLift。
            } else {
              planarVoxelElev[i] = planarPointElev[i][quantileID];//否则，将平面体素最小高度设置为分位数点的高度
            }
          }
        }
      } else {//不进行排序
        for (int i = 0; i < planarVoxelNum; i++) {
          int planarPointElevSize = planarPointElev[i].size();
          if (planarPointElevSize > 0) {
            float minZ = 1000.0;
            int minID = -1;
            for (int j = 0; j < planarPointElevSize; j++) {
              if (planarPointElev[i][j] < minZ) {
                minZ = planarPointElev[i][j];
                minID = j;//找到第i个体素中最低高度的索引j
              }
            }
 
            if (minID != -1) {
              planarVoxelElev[i] = planarPointElev[i][minID];//将该体素高度设置为该体素最低的点的高度
            }
          }
        }
      }
 
      //对车辆周围的terrainCloudSize点云以planarVoxelSize细分网格
      terrainCloudElev->clear();//清空 terrainCloudElev 指向的点云数组数据
      int terrainCloudElevSize = 0;
      for (int i = 0; i < terrainCloudSize; i++) { //车辆相邻5个terrainVoxelsize网格范围内的点
        point = terrainCloud->points[i];
        if (point.z - vehicleZ > minRelZ && point.z - vehicleZ < maxRelZ) {
          int indX = int((point.x - vehicleX + planarVoxelSize / 2) / //类似于上面的网格划分，以planarVoxelSize的长度细分网格，planarVoxelNum=（10/0.2）+1=51
                         planarVoxelSize) +
                     planarVoxelHalfWidth;
          int indY = int((point.y - vehicleY + planarVoxelSize / 2) / //获得该点对应的体素坐标
                         planarVoxelSize) +
                     planarVoxelHalfWidth;
 
          if (point.x - vehicleX + planarVoxelSize / 2 < 0)
            indX--;
          if (point.y - vehicleY + planarVoxelSize / 2 < 0)
            indY--;
 
          if (indX >= 0 && indX < planarVoxelWidth && indY >= 0 &&
              indY < planarVoxelWidth) {//ind在索引范围内
            if (planarVoxelDyObs[planarVoxelWidth * indX + indY] <//该点非障碍点
                    minDyObsPointNum ||
                !clearDyObs) {
              float disZ =point.z - planarVoxelElev[planarVoxelWidth * indX + indY];//点云中某点的 Z 坐标与对应体素估算的地面高度（体素最低高度）之间的差值
              if (considerDrop)
                disZ = fabs(disZ);
              int planarPointElevSize =planarPointElev[planarVoxelWidth * indX + indY].size();//该planarVoxel体素的数量
              if (disZ >= 0 && disZ < vehicleHeight && //该点高度>0,小于车辆高度，其为有高度价值点，记录
                  planarPointElevSize >= minBlockPointNum) {
                terrainCloudElev->push_back(point);//将该点云存入点云数组terrainCloudElev
                terrainCloudElev->points[terrainCloudElevSize].intensity = disZ;//将该点的强度值设置为点云距离地面高度
 
                terrainCloudElevSize++;//terrainCloudElev数组下标+1
              }
            }
          }
        }
      }
      
      //在点云数据中标记出缺少数据的区域，以便将这些区域视为潜在的障碍物
      if (noDataObstacle && noDataInited == 2) {//noDataObstacle==true(启动无数据处为障碍模式)，noDataInited==2部分点云丢失
        for (int i = 0; i < planarVoxelNum; i++) {//遍历体素planarVoxel，若标记点数少于阈值的体素为边缘体素
          int planarPointElevSize = planarPointElev[i].size();//planarPointElev第i个平面体素中存储符合要求的点云高程数据
          if (planarPointElevSize < minBlockPointNum) {//如果该planar体素的高程信息很少（点云数据缺失）
            planarVoxelEdge[i] = 1;//将该边缘体素标记设置为1（认为该缺失数据的体素为边缘/障碍体素）
          }
        }
 
        for (int noDataBlockSkipCount = 0;
             noDataBlockSkipCount < noDataBlockSkipNum;//如果该planarVoxelEdge[i]，第i个体素被判定为边缘体素，需要被评估（遍历）noDataBlockSkipNum次
             noDataBlockSkipCount++) {
          for (int i = 0; i < planarVoxelNum; i++) {//遍历planarVoxel体素网格（51）
            if (planarVoxelEdge[i] >= 1) {//如果该体素为边缘/障碍
              int indX = int(i / planarVoxelWidth);//取整得x坐标(ind=planarVoxelWidth * indX + indY)
              int indY = i % planarVoxelWidth;
              bool edgeVoxel = false;
              for (int dX = -1; dX <= 1; dX++) {//对该体素以及周围8个体素进行遍历（3*3）
                for (int dY = -1; dY <= 1; dY++) {
                  if (indX + dX >= 0 && indX + dX < planarVoxelWidth &&
                      indY + dY >= 0 && indY + dY < planarVoxelWidth) {//使得ind不越界
                    if (planarVoxelEdge[planarVoxelWidth * (indX + dX) + indY +
                                        dY] < planarVoxelEdge[i]) {//通过查看体素 i 周围的体素来判断它是否真的是一个边缘体素
                      edgeVoxel = true;//如果周围有任何一个体素的边缘标记比当前体素小，则认为当前体素是真正的边缘体素
                    }
                  }
                }
              }
 
              if (!edgeVoxel)
                planarVoxelEdge[i]++;//当前体素不是边缘体素，则增加其边缘标记的值（进行重新评估，直到遍历noDataBlockSkipNum次看最终评估结果）
            }
          }
        }
 
        //对识别为无数据区域（边缘），虚拟障碍物，添加到体素中（将无数据边缘视为障碍）
        for (int i = 0; i < planarVoxelNum; i++) {//planarVoxel体素遍历
          if (planarVoxelEdge[i] > noDataBlockSkipNum) {//如果一个体素的边缘标记大于noDataBlockSkipNum，这个体素周围没有足够的数据（车辆一次雷达更新时间移动超过传感器检测范围）
            int indX = int(i / planarVoxelWidth);
            int indY = i % planarVoxelWidth;
 
            point.x =
                planarVoxelSize * (indX - planarVoxelHalfWidth) + vehicleX;//该体素位于map坐标系下的位置
            point.y =
                planarVoxelSize * (indY - planarVoxelHalfWidth) + vehicleY;
            point.z = vehicleZ;//该点车辆的里程计高度
            point.intensity = vehicleHeight;//该点强度为车辆实际高度，虚拟障碍物高度
 
            point.x -= planarVoxelSize / 4.0;//将点移动到体素左下角（体素中心-体素边长的1/4）
            point.y -= planarVoxelSize / 4.0;
            terrainCloudElev->push_back(point);
 
            point.x += planarVoxelSize / 2.0;//将虚拟障碍点移动到体素右下角，并添加到terrainCloudElev
            terrainCloudElev->push_back(point);
 
            point.y += planarVoxelSize / 2.0;//移动到右上
            terrainCloudElev->push_back(point);
 
            point.x -= planarVoxelSize / 2.0;//移动到左上
            terrainCloudElev->push_back(point);
          }
        }
      }
 
      clearingCloud = false;
 
      // publish points with elevation
      sensor_msgs::PointCloud2 terrainCloud2;
      pcl::toROSMsg(*terrainCloudElev, terrainCloud2);//将terrainCloudElev转化为ros消息格式
      terrainCloud2.header.stamp = ros::Time().fromSec(laserCloudTime);
      terrainCloud2.header.frame_id = "map";
      pubLaserCloud.publish(terrainCloud2);//将地形信息，通过pubLaserCloud对象发布到map坐标系下的terrain_map话题中
    }
 
    status = ros::ok();
    rate.sleep();
  }
 
  return 0;
}