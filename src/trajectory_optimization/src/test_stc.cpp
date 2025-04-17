#include "test_stc.hpp"

CorridorGenerator::CorridorGenerator(){
  // 发布安全走廊的可视化消息
  m_ellipsoid_array_pub = nh_.advertise<decomp_ros_msgs::EllipsoidArray>(
      "ellipsoid_array", 1, true);
  m_polyhedron_array_pub = nh_.advertise<decomp_ros_msgs::PolyhedronArray>(
      "polyhedron_array", 1, true);

  // 订阅地图消息
  sub_gridmap_ = nh_.subscribe<nav_msgs::OccupancyGrid>(
      "/map", 1, &CorridorGenerator::mapCallback, this);

  // 订阅路径消息
  sub_path_ = nh_.subscribe<nav_msgs::Path>(
      "/move_base/TebLocalPlannerROS/global_plan", 1, &CorridorGenerator::pathCallback, this);

  // 定时器用于生成安全走廊
  corridor_generate_timer = nh_.createTimer(
      ros::Duration(0.05),
      boost::bind(&CorridorGenerator::corridor_generate_timer_cb, this, _1));
}

CorridorGenerator::~CorridorGenerator(){

}

void CorridorGenerator::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg) {
  m_obs2d.clear();
  double resolution = msg->info.resolution;
  double origin_x = msg->info.origin.position.x;
  double origin_y = msg->info.origin.position.y;

  // 遍历地图中的每个栅格，提取障碍物点
  for (unsigned int i = 0; i < msg->info.width; ++i) {
    for (unsigned int j = 0; j < msg->info.height; ++j) {
      int index = i + j * msg->info.width;
      if (msg->data[index] > 50) { // 阈值判断是否为障碍物
        double x = origin_x + i * resolution;
        double y = origin_y + j * resolution;
        m_obs2d.push_back(Eigen::Vector2d(x, y));
      }
    }
  }
}

void CorridorGenerator::pathCallback(const nav_msgs::Path::ConstPtr &msg) {
  m_discrete_path.clear();
  for (const auto &pose : msg->poses) {
    Eigen::Vector2d point(pose.pose.position.x, pose.pose.position.y);
    m_discrete_path.push_back(point);
  }
}

void CorridorGenerator::getDiscretePath(double interval) {
  if (m_discrete_path.empty()) {
    return;
  }

  vec_E<Eigen::Vector2d> discrete_path;
  discrete_path.push_back(m_discrete_path.front());

  auto calDis = [](const Eigen::Vector2d &a, const Eigen::Vector2d &b) {
    return (a - b).norm();
  };

  for (size_t i = 1; i < m_discrete_path.size(); ++i) {
    if (calDis(m_discrete_path[i], discrete_path.back()) >= interval) {
      discrete_path.push_back(m_discrete_path[i]);
    }
  }

  discrete_path.push_back(m_discrete_path.back());
  m_discrete_path = discrete_path;
}

void CorridorGenerator::generateCorridor() {
  if (m_discrete_path.empty() || m_obs2d.empty()) {
    ROS_WARN("Waiting for map and path...");
    return;
  }

  // 设置障碍物点
  m_decomp_util.set_obs(m_obs2d);

  // 设置局部搜索范围
  m_decomp_util.set_local_bbox(Vec2f(1, 2));

  // 基于离散路径生成安全走廊
  m_decomp_util.dilate(m_discrete_path);

  // 获取生成的多面体和线性约束
  m_polyhedron2d = m_decomp_util.get_polyhedrons();
  m_constraints = m_decomp_util.get_constraints();

}

// 约束路径

void CorridorGenerator::optimizePath(const Eigen::Vector2d &goal) {
  if (m_constraints.empty() || m_discrete_path.empty()) {
    ROS_WARN("No constraints or path available for optimization.");
    return;
  }

  // 提取线性约束矩阵 A 和 b
  size_t num_constraints = m_constraints.size();
  Eigen::MatrixXd A(num_constraints, 2); // 假设二维空间
  Eigen::VectorXd b(num_constraints);

  for (size_t i = 0; i < num_constraints; ++i) {
    A.row(i) = m_constraints[i].A().transpose(); // 获取约束的法向量
    b(i) = m_constraints[i].b()(0);     // 获取约束的偏移量
  }

  // 定义目标函数的二次项和一次项
  Eigen::MatrixXd P = Eigen::MatrixXd::Identity(2, 2); // 单位矩阵
  Eigen::VectorXd q = Eigen::VectorXd::Zero(2); // 最小化路径长度

  // 转换为 OSQP 格式
  int n = 2; // 变量维度
  int m = num_constraints; // 约束数量

// OSQP 数据
std::unique_ptr<OSQPSettings> settings = std::make_unique<OSQPSettings>();
std::unique_ptr<OSQPData> data = std::make_unique<OSQPData>();

data->n = n;
data->m = m;
data->P = csc_matrix(n, n, P.nonZeros(), P.valuePtr(), P.innerIndexPtr(), P.outerIndexPtr());
data->q = q.data();
data->A = csc_matrix(m, n, A.nonZeros(), A.valuePtr(), A.innerIndexPtr(), A.outerIndexPtr());
data->l = nullptr; // 无下界
data->u = b.data(); // 上界

// 设置默认参数
osqp_set_default_settings(settings.get());
settings->alpha = 1.0; // 松弛因子

// 初始化 OSQP
std::unique_ptr<OSQPWorkspace, decltype(&osqp_cleanup)> work(nullptr, osqp_cleanup);
if (osqp_setup(&work, data.get(), settings.get()) != 0) {
  ROS_ERROR("Failed to set up OSQP solver.");
  return;
}

// 求解
osqp_solve(work.get());

// 获取结果
Eigen::Vector2d optimized_point(work->solution->x[0], work->solution->x[1]);
ROS_INFO_STREAM("Optimized point: " << optimized_point.transpose());
}

void CorridorGenerator::corridor_generate_timer_cb(const ros::TimerEvent &e) {
  getDiscretePath(2.0); // 离散化路径，间隔为 2.0
  generateCorridor();   // 生成安全走廊
  displayCorridor();    // 可视化安全走廊
}

void CorridorGenerator::displayCorridor() {
  if (m_elliposid2d.empty() || m_polyhedron2d.empty()) {
    return;
  }

  // 发布椭圆体表示的安全走廊
  decomp_ros_msgs::EllipsoidArray es_msg =
      DecompROS::ellipsoid_array_to_ros(m_elliposid2d);
  es_msg.header.frame_id = "map";
  m_ellipsoid_array_pub.publish(es_msg);

  // 发布多面体表示的安全走廊
  decomp_ros_msgs::PolyhedronArray poly_msg =
      DecompROS::polyhedron_array_to_ros(m_polyhedron2d);
  poly_msg.header.frame_id = "map";
  m_polyhedron_array_pub.publish(poly_msg);

  // 可视化优化路径
  visualization_msgs::Marker optimized_path_marker;
  optimized_path_marker.header.frame_id = "map";
  optimized_path_marker.type = visualization_msgs::Marker::LINE_STRIP;
  optimized_path_marker.scale.x = 0.1;
  optimized_path_marker.color.r = 1.0;
  optimized_path_marker.color.a = 1.0;

  for (const auto &point : m_optimized_path) {
    geometry_msgs::Point p;
    p.x = point.x();
    p.y = point.y();
    p.z = 0.0;
    optimized_path_marker.points.push_back(p);
  }

  m_optimized_path_pub.publish(optimized_path_marker);
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "stc");
  CorridorGenerator CorridorGenerator;
  ros::spin();
  return 0;
}
