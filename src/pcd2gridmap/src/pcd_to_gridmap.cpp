 #include "pcd_to_gridmap.hpp"

PCDToGridMap::PCDToGridMap():
    pr_nh_("~"),
    transform_(Eigen::Affine3d::Identity()){

    pr_nh_.param<std::string>("map/pcd_file", pcd_file_, "");
    pr_nh_.param<double>("map/resolution", map_resolution_, 0.05);
    
    pr_nh_.param<double>("transform/x", x_, 0.0);
    pr_nh_.param<double>("transform/y", y_, 0.0);
    pr_nh_.param<double>("transform/z", z_, 0.0);
    pr_nh_.param<double>("transform/roll", roll_, 0.0);
    pr_nh_.param<double>("transform/pitch", pitch_, 0.0);
    pr_nh_.param<double>("transform/yaw", yaw_, 0.0);

    pr_nh_.param<double>("filter/thre_z_min", thre_z_min_, -0.5);
    pr_nh_.param<double>("filter/thre_z_max", thre_z_max_, 0.5);
    pr_nh_.param<double>("filter/thre_radius", thre_radius_, 0.5);
    pr_nh_.param<int>("filter/thre_count", thre_count_, 5);

    ROS_INFO_STREAM("PCD file: " << pcd_file_);

    pcd_cloud_ = PointCloudT::Ptr(new PointCloudT);
    filtered_passcloud_ = PointCloudT::Ptr(new PointCloudT);
    filtered_radiuscloud_ = PointCloudT::Ptr(new PointCloudT);

    pub_map_ = nh_.advertise<nav_msgs::OccupancyGrid>("/map", 1, true);
    pub_pcd_ = nh_.advertise<sensor_msgs::PointCloud2>("/filtered_pcd", 1, true);

    timer_ = nh_.createTimer(ros::Duration(1.0), std::bind(&PCDToGridMap::publishCb, this));
    
    loadPCD();
    pcdFilter();
    buildMap(filtered_radiuscloud_, map_);
}

PCDToGridMap::~PCDToGridMap() {
}

void PCDToGridMap::publishCb(){
    sensor_msgs::PointCloud2 cloud_out;
    pcl::toROSMsg(*filtered_passcloud_, cloud_out);
    cloud_out.header.frame_id = "map";

    pub_map_.publish(map_);
    pub_pcd_.publish(cloud_out);
}

void PCDToGridMap::loadPCD(){
    // load PCD file
    if(pcd_file_.empty()){
        ROS_ERROR("PCD file path is empty!");
        return;
    }

    if (pcl::io::loadPCDFile<PointT>(pcd_file_, *pcd_cloud_) == -1) {
        ROS_ERROR_STREAM("Failed to load PCD file: " << pcd_file_);
        return; 
    }    

    ROS_INFO_STREAM("Loaded PCD file: " << pcd_file_ << " with " << pcd_cloud_->size() << " points.");

    // deg to rad
    roll_ = roll_ * M_PI / 180.0;
    pitch_ = pitch_ * M_PI / 180.0;
    yaw_ = yaw_ * M_PI / 180.0;

    // apply transformation
    transform_.translation() = Eigen::Vector3d(x_, y_, z_);
    Eigen::AngleAxisd roll_rotation(roll_, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitch_rotation(pitch_, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yaw_rotation(yaw_, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond q = yaw_rotation * pitch_rotation * roll_rotation;
    transform_.linear() = q.toRotationMatrix();

    pcl::transformPointCloud(*pcd_cloud_, *pcd_cloud_, transform_);
}

void PCDToGridMap::pcdFilter(){
    if(!pcd_cloud_ || pcd_cloud_->empty()){
        ROS_ERROR_STREAM("PCD cloud is empty!");
        return;
    }

    pcl::PassThrough<PointT> pass;
    pass.setInputCloud(pcd_cloud_);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(thre_z_min_, thre_z_max_);
    pass.setNegative(false);
    pass.filter(*filtered_passcloud_);

    ROS_INFO_STREAM("Pass through filtered PCD : " << " with " << filtered_passcloud_->size() << " points.");

    pcl::RadiusOutlierRemoval<PointT> outrem;
    outrem.setInputCloud(filtered_passcloud_);
    outrem.setRadiusSearch(thre_radius_);
    outrem.setMinNeighborsInRadius(thre_count_);
    outrem.setNegative(false);
    outrem.filter(*filtered_radiuscloud_);

    ROS_INFO_STREAM("Radius outier filtered PCD : " << " with " << filtered_radiuscloud_->size() << " points.");
}


void PCDToGridMap::buildMap(const PointCloudT::Ptr &cloud, nav_msgs::OccupancyGrid &map){
    map.header.frame_id = "map";
    map.header.stamp = ros::Time::now();

    map.info.resolution = map_resolution_;
    map.info.map_load_time = ros::Time::now();

    double min_x = std::numeric_limits<double>::max();
    double min_y = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::lowest();
    double max_y = std::numeric_limits<double>::lowest();

    if(cloud->points.empty() || !cloud){
        ROS_ERROR_STREAM("Cloud is empty!");
        return;
    }

    for (const auto &point : cloud->points) {
        min_x = std::min(min_x, static_cast<double>(point.x));
        min_y = std::min(min_y, static_cast<double>(point.y));
        max_x = std::max(max_x, static_cast<double>(point.x));
        max_y = std::max(max_y, static_cast<double>(point.y));
    }

    map.info.origin.position.x = min_x;
    map.info.origin.position.y = min_y;
    map.info.origin.position.z = 0.0;
    map.info.origin.orientation.w = 1.0;
    map.info.origin.orientation.x = 0.0;
    map.info.origin.orientation.y = 0.0;
    map.info.origin.orientation.z = 0.0;

    map.info.width = std::ceil((max_x - min_x) / map_resolution_);
    map.info.height = std::ceil((max_y - min_y) / map_resolution_);
    map.data.assign(map.info.width * map.info.height, 0);

    for(const auto &point : cloud->points){
        int i = std::floor((point.x - min_x) / map_resolution_);
        int j = std::floor((point.y - min_y) / map_resolution_);

        if(i >= 0 && i < map.info.width && j >= 0 && j < map.info.height){
            int index = i + j * map.info.width;
            if(index >= 0 && index < map.data.size()){
                map.data[index] = 100; // mark as occupied
            }
        }
    }

    ROS_INFO_STREAM("Map built with size: " << map.info.width << " x " << map.info.height);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "pcd_to_gridmap");

    PCDToGridMap pcd_to_gridmap;

    ros::spin();
    return 0;
}



