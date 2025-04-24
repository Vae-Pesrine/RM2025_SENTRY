#include <tf2/utils.h>
#include "lbfgs.hpp"
#include "trajectory.hpp"
#include "stc_gen.hpp"
#include "test_stc.hpp"

template <int Dim>
decomp_ros_msgs::PolyhedronArray polyhedron_array_to_ros(const vec_E<Polyhedron<Dim>>& vs, int delta = 1){
    decomp_ros_msgs::PolyhedronArray msg;
    msg.header.frame_id = "map";
    msg.header.stamp = ros::Time::now();
    int i = 1;
    for (const auto &v : vs){
        if(i == delta){
            msg.polyhedrons.push_back(DecompROS::polyhedron_to_ros(v)); 
            i = 0;   
        }
        i++;
    }
    return msg;
}

CorridorGenerator::CorridorGenerator():
  max_radius_(10.0){
  // 发布安全走廊的可视化消息
   pub_opt_path = nh_.advertise<nav_msgs::Path>("/opt_path", 1, true);
  m_ellipsoid_array_pub = nh_.advertise<decomp_ros_msgs::EllipsoidArray>(
      "ellipsoid_array", 1, true);
  m_polyhedron_array_pub = nh_.advertise<decomp_ros_msgs::PolyhedronArray>(
      "polyhedron_array", 1, true);

      // 新增：发布优化路径的可视化消息
  m_optimized_path_pub = nh_.advertise<visualization_msgs::Marker>(
    "optimized_path", 1, true);

  // 订阅地图消息
  sub_gridmap_ = nh_.subscribe<nav_msgs::OccupancyGrid>("/map", 1, &CorridorGenerator::mapCallback, this);

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

void CorridorGenerator::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &costmap) {
  costmap_ = costmap;
  
  // m_obs2d.clear();
  // double resolution = costmap->info.resolution;
  // double origin_x = costmap->info.origin.position.x;
  // double origin_y = costmap->info.origin.position.y;

  // // 遍历地图中的每个栅格，提取障碍物点
  // for (unsigned int i = 0; i < costmap->info.width; ++i) {
  //   for (unsigned int j = 0; j < costmap->info.height; ++j) {
  //     int index = i + j * costmap->info.width;
  //     if (costmap->data[index] > 50) { // 阈值判断是否为障碍物
  //       double x = origin_x + i * resolution;
  //       double y = origin_y + j * resolution;
  //       m_obs2d.push_back(Eigen::Vector2d(x, y));
  //     }
  //   }
  // }
}

void CorridorGenerator::pathCallback(const nav_msgs::Path::ConstPtr &msg) {
  path_ = *msg;
  pieceNum_ = 0;
  for(auto &pose : msg->poses){
    pieceNum_++;
    path_stc_.emplace_back(pose.pose.position.x, pose.pose.position.y);
  }
  optDim_ = 2 * (pieceNum_ - 1);

  m_discrete_path.clear();
  for (const auto &pose : msg->poses) {
    Eigen::Vector2d point(pose.pose.position.x, pose.pose.position.y);
    m_discrete_path.push_back(point);
  }
}

// 根据前端路径得到离散化的路径
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

void CorridorGenerator::DecompVel(const double theta, const double vel, double &vx, double &vy){
  const double epsi = 1e-8;
  double vt = std::abs(vel) < epsi?epsi:std::abs(vel);
  vx = vt*std::cos(theta);
  vy = vt*std::sin(theta);
}

void CorridorGenerator::positiveSmoothedL1(const double &x, double &f, double &df){
  const double pe = 1.0e-4;
  const double half = 0.5 * pe;
  const double f3c = 1.0 / (pe * pe);
  const double f4c = -0.5 * f3c / pe;
  const double d2c = 3.0 * f3c;
  const double d3c = 4.0 * f4c;

  if (x < pe)
  {
      f = (f4c * x + f3c) * x * x * x;
      df = (d3c * x + d2c) * x * x;
  }
  else
  {
      f = x - half;
      df = 1.0;
  }
  return;
}


inline double CorridorGenerator::costFunctional(void *ptr, const Eigen::VectorXd &x, Eigen::VectorXd &grad) {
  CorridorGenerator &obj = *(CorridorGenerator *)ptr;
  const int dimXi = obj.optDim_;
  const int N = obj.pieceNum_;
  // const double weightT = 1.0;

  obj.iter++;

  Eigen::Matrix<double, 6, 1> beta0, beta1, beta2, beta3, beta4;

  auto &ts = obj.ts_;

  double t2,t3,t4,t5;
  Eigen::Vector2d sigma, dsigma, ddsigma, dddsigma, ddddsigma;

  double gCur = 0.0;
  Eigen::Matrix<double, -1, 2> gradByC(6*N,2);
  gradByC.setZero();

  Eigen::Map<const Eigen::Matrix<double,2, -1, Eigen::RowMajor>> inPos(x.data(),2,dimXi/2);
  Eigen::Map<Eigen::Matrix<double, 2, -1, Eigen::RowMajor>> gradByP(grad.data(),2,dimXi/2);
  gradByP.setZero();

  Eigen::Matrix2d B;
  B << 0,-1,
       1, 0;

  obj.ts_ = Eigen::VectorXd::Constant(obj.pieceNum_,1); 
  obj.opt_.setParameters(inPos.transpose(),obj.ts_);
  
  for(int i = 0;i< N;i++){
    if(obj.hpolys.size() <= i || obj.hpolys[i].cols() < 3 || obj.hpolys[i].rows() == 0) {
      ROS_ERROR_STREAM("hpolys[" << i << "] is invalid! size=" << obj.hpolys.size()
                       << ", cols=" << (obj.hpolys.size() > i ? obj.hpolys[i].cols() : -1)
                       << ", rows=" << (obj.hpolys.size() > i                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 ? obj.hpolys[i].rows() : -1));
      continue;
  }    
      int Ki = 30; // 一段ki个点，包括端点
      double step = ts[i]/(Ki-1);

      const Eigen::Matrix<double, 6, 2> &c = obj.opt_.getCoeffs().middleRows(6*i,6);

      for(auto t1 = 0.0;t1 <= ts[i];t1+=step){
          if(i == 0 || i == N-1){
              t1+=step;
          }
          t2 = t1*t1;
          t3 = t2*t1;
          t4 = t2*t2;
          t5 = t4*t1;

          beta0 << 1.0,t1,t2,t3,t4,t5;
          beta1 << 0.0, 1.0, 2.0 * t1, 3.0 * t2, 4.0 * t3, 5.0 * t4;
          beta2 << 0.0, 0.0, 2.0, 6.0 * t1, 12.0 * t2, 20.0 * t3;
          beta3 << 0.0, 0.0, 0.0, 6.0, 24.0 * t1, 60.0 * t2;
          beta4 << 0.0, 0.0, 0.0, 0.0, 24.0, 120 * t1;

          sigma = c.transpose() * beta0;
          dsigma = c.transpose() * beta1;
          ddsigma = c.transpose() * beta2;
          dddsigma = c.transpose() * beta3;
          ddddsigma = c.transpose() * beta4;

          double z2 = ddsigma.transpose() * B * dsigma;
          auto vel = dsigma.norm();
          double epis = 1e-10;
          auto vel32 = std::pow(vel,3)+epis;
          // auto vel62 = vel32*vel32;
          auto vel52 = std::pow(vel,5)+epis;

          double w_cur = 1;
          double w_cor = 1;

          Eigen::Matrix<double, 6, 2> gradByCT_t;
          gradByCT_t.setZero();

          //corridor 
          double violaCorPena = 0.0,violaCorPenaD = 0.0;
          auto resu = obj.hpolys[i].leftCols(2)*sigma-obj.hpolys[i].rightCols(1);
          for(int j = 0; j<resu.rows();j++){
              if(resu(j)>0.0){
                  positiveSmoothedL1(resu(j), violaCorPena, violaCorPenaD);
                  gCur+= w_cor * violaCorPena;
                  gradByCT_t += w_cor*beta0*obj.hpolys[i].block(j,0,1,2)*violaCorPenaD;
              }
          }

          // kappa


          double km =  1 / obj.max_radius_;
          auto violaCurL = z2/vel32-km;
          auto violaCurR = -z2/vel32-km;
          double violaCurPenaL = 0,violaCurPenaDL = 0;
          double violaCurPenaR = 0,violaCurPenaDR = 0;

          if(violaCurL>0.0){
              positiveSmoothedL1(violaCurL, violaCurPenaL, violaCurPenaDL);
              gCur+=w_cur*violaCurPenaL;
              gradByCT_t += w_cur*(beta1*(ddsigma.transpose()*B/vel32-3*z2*dsigma.transpose()/vel52)+
              beta2*dsigma.transpose()*B.transpose()/vel32 )*violaCurPenaDL;

          }

          if(violaCurR>0.0){
              positiveSmoothedL1(violaCurR, violaCurPenaR, violaCurPenaDR);
              gCur+=w_cur*violaCurPenaR;
              gradByCT_t += w_cur * -(beta1*(ddsigma.transpose()*B/vel32-3*z2*dsigma.transpose()/vel52)+
              beta2*dsigma.transpose()*B.transpose()/vel32 )*violaCurPenaDR;
          }
          gradByC.middleRows(6*i,6) += gradByCT_t; 
      }
  }
          
  obj.opt_.propogateGrad(gradByC,gradByP);
  return gCur;
}

double CorridorGenerator::Lbfgs(Eigen::VectorXd &xi){
  lbfgs::lbfgs_parameter_t opt_params;
  opt_params.max_linesearch = 128;
  opt_params.min_step = 1e-32;
  opt_params.mem_size = 128;
  opt_params.past = 3;
  opt_params.delta = 1.0e-6;
  opt_params.g_epsilon = 1.0e-5;
  opt_params.max_iterations = 10000;

  double cost = 0.0;
  int reslut_code = lbfgs::lbfgs_optimize(xi, cost,
                          &CorridorGenerator::costFunctional,
                          nullptr,
                          nullptr,
                          this,
                          opt_params);
  printf("lbfgs-ret: %s\n",lbfgs::lbfgs_strerror(reslut_code));
  return cost;
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
void CorridorGenerator::optimizePathWithCorridor() {
  if(path_.poses.size() < 2) {
    ROS_ERROR("path_.poses.size() < 2, cannot optimize!");
    return;
  }
  if(path_stc_.size() < 2) {
    ROS_ERROR("path_stc_ size < 2, cannot optimize!");
    return;
  }
  if(pieceNum_ != static_cast<int>(path_.poses.size())) {
    ROS_ERROR("pieceNum_ (%d) != path_.poses.size() (%zu)", pieceNum_, path_.poses.size());
    return;
  }

  VecE<Polyhedron<2>> ploy_vis;
  std::cout << "stc begins" << std::endl;
  STCGen(costmap_, path_stc_, hpolys, ploy_vis);
  decomp_ros_msgs::PolyhedronArray poly_msg = polyhedron_array_to_ros(ploy_vis, 1);
  m_ellipsoid_array_pub.publish(poly_msg);
  std::cout << "stc ends" << std::endl;

  double vsx, vsy, vex, vey;
  DecompVel(tf2::getYaw(path_.poses.front().pose.orientation), 10, vsx, vsy);
  DecompVel(tf2::getYaw(path_.poses.back().pose.orientation), 10, vex, vey);  
  
  Eigen::Matrix<double, 3, 2> headState;
  Eigen::Matrix<double, 3, 2> tailState;
  headState.row(0) = Eigen::Vector2d(path_.poses.front().pose.position.x,
                                      path_.poses.front().pose.position.y).transpose();
  headState.row(1) = Eigen::Vector2d(vsx, vsy).transpose();
  headState.row(2) = Eigen::Vector2d(0, 0).transpose();

  tailState.row(0) = Eigen::Vector2d(path_.poses.back().pose.position.x,
                                      path_.poses.back().pose.position.y).transpose();
  tailState.row(1) = Eigen::Vector2d(vex, vey).transpose();
  tailState.row(2) = Eigen::Vector2d(0, 0).transpose();

  opt_.setConditions(headState, tailState, pieceNum_);

  Eigen::Matrix<double, 2, -1> inPs(2, pieceNum_ - 1);
  inPs.setZero();
  for(int i = 1; i < pieceNum_; ++i){
    inPs.col(i-1) = Eigen::Vector2d(path_.poses[i].pose.position.x, path_.poses[i].pose.position.y);
      
  }

  Eigen::VectorXd xi(optDim_);
  xi.topRows(pieceNum_ - 1) = inPs.row(0).transpose();
  xi.bottomRows(pieceNum_ - 1) = inPs.row(1).transpose();

  auto time1 = ros::Time::now();
  auto optcost = Lbfgs(xi);
  auto dt = ros::Time::now() - time1;
  printf("opt time: %f ms\n",dt.toSec()*1000);
  printf("opt cost: %.10f iter: %d\n",optcost,iter);

  inPs.row(0) = xi.topRows(pieceNum_ - 1).transpose();
  inPs.row(1) = xi.bottomRows(pieceNum_ - 1).transpose();

  opt_.setConditions(headState, tailState, pieceNum_);
  ts_ = Eigen::VectorXd::Constant(pieceNum_, 1);
  opt_.setParameters(inPs.transpose(), ts_);

  Trajectory<2, 5> traj;
  opt_.getTrajectory(traj);

  std::vector<Eigen::Vector2d> trajs;
  for(double t = 0.0; t <= traj.getPieceNum()*1; t+=0.05){
    trajs.emplace_back(traj.getPos(t));
  }

  nav_msgs::Path path;
  path.header.frame_id = "map";
  path.header.stamp = ros::Time::now();
  path.poses.resize(trajs.size());
  for(size_t i = 0; i < trajs.size(); i++){
    path.poses[i].pose.position.x = trajs[i].x();
    path.poses[i].pose.position.y = trajs[i].y();
    path.poses[i].header.frame_id = "map";
    path.poses[i].header.stamp = ros::Time::now();
  }
  pub_opt_path.publish(path);

}

void CorridorGenerator::corridor_generate_timer_cb(const ros::TimerEvent &e) {
  getDiscretePath(2.0); // 离散化路径，间隔为 2.0
  optimizePathWithCorridor(); // 优化路径
  // displayCorridor();    // 可视化安全走廊
}

// 生成安全走廊
void CorridorGenerator::STCGen(const nav_msgs::OccupancyGridConstPtr map, const std::vector<Eigen::Vector2d> &path,
  VecE<Eigen::MatrixX3d> &hpoly, VecE<Polyhedron<2>> &ploys_vis) {
  if(path.size() < 2) return;  // 防止resize极大
  hpoly.resize(path.size()-1);
  ploys_vis.resize(path.size()-1);
  for(size_t i = 0; i < path.size()-1; i++) {
      auto center = (path[i]+path[i+1])/2;
      double dist = (path[i]-path[i+1]).norm();
      Eigen::Vector2d box(dist,dist);
      VecE<Eigen::Vector2d> vec_map;
      double side = std::pow(std::max(box.x(),box.y())*2,2);
      Map2Vec2D(costmap_, center.x(), center.y(), side, side, vec_map);
      stc_gen::STCGen::ConvexHull({path[i], path[i+1]}, vec_map, hpoly[i], ploys_vis[i], dist, dist);
  }
}

// 从栅格地图提取障碍物索引
void CorridorGenerator::Map2Vec2D(const nav_msgs::OccupancyGridConstPtr costmap,const double center_x, const double center_y, 
  const double roi_w,const double roi_h, VecE<Eigen::Vector2d> &vec_map){
      vec_map.clear();
      auto resolution = costmap->info.resolution;
      auto o_x = costmap->info.origin.position.x;
      auto o_y = costmap->info.origin.position.y;

      // size_t i = 0;

      auto x_s = static_cast<int>((center_x - roi_w/2-o_x)/resolution);
      auto y_s = static_cast<int>((center_y - roi_h/2-o_y)/resolution);
      x_s = x_s < 0?0:x_s;
      y_s = y_s < 0?0:y_s;
      auto x_e = static_cast<int>((center_x + roi_w/2-o_x)/resolution);
      auto y_e = static_cast<int>((center_y + roi_h/2-o_y)/resolution);
      x_e = x_e < costmap->info.width?x_e:costmap->info.width-1;
      y_e = y_e < costmap->info.height?y_e:costmap->info.height-1;

      for (int i = x_s;i<= x_e;i++)
      for (int j = y_s;j<= y_e;j++) {
          if (costmap->data[j*costmap->info.width+i] > 1){
              vec_map.emplace_back((i+0.5) * resolution, (j+0.5) * resolution); 
          }
      }
  }


int main(int argc, char *argv[])
{
  ros::init(argc, argv, "stc");
  CorridorGenerator CorridorGenerator;
  ros::spin();
  return 0;
}
