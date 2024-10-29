//created ny Jia GuangYi
#ifndef UNSCENTED_KALMAN_FILTER_HPP
#define UNSCENTED_KALMAN_FILTER_HPP

#include <random>
#include <Eigen/Dense>

namespace sentry_localization{

template<typename T, class System>
class UnscentedKalmanFilterX
{
  using VectorXt = Eigen::Matrix<T, Eigen::Dynamic, 1>;
  using MatrixXt = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>;
public :
  UnscentedKalmanFilterX(const System system, int state_dim, int input_dim, int measurement_dim, const MatrixXt& process_noise, const MatrixXt& measurement_noise, const VectorXt& mean, const MatrixXt& cov)
    : state_dim(state_dim),                                        
      input_dim(input_dim),                                       
      measurement_dim(measurement_dim),                             
      N(state_dim),                           
      M(input_dim),
      K(measurement_dim),
      S(2 * state_dim + 1),                                         
      mean(mean),                                                   
      cov(cov),                                                     
      system(system),                                               
      process_noise(process_noise),                                 
      measurement_noise(measurement_noise),                         
      lambda(1),                                                    
      normal_dist(0.0, 1.0)                                         
  {
    weights.resize(2 * N + 1, 1);                                   
    extension_weights.resize(2 * (N + K) + 1, 1);                   
    sigma_points.resize(2 * N + 1, N);                              
    extension_sigma_points.resize(2 * (N + K) + 1, N + K);          
    expected_measurements.resize(2 * (N + K) + 1, K);               

    //初始化sigma点的权重
    weights[0] = lambda / (lambda + N);
    for(int i = 0; i < 2 * N + 1; ++i)
    {
      weights[i] = 1 / (2 * (N + lambda));
    }

    // 初始化扩展sigma点的权重
    extension_weights[0] = lambda / (N + K + lambda);
    for(int i = 0; i < 2 * (N + K) + 1; ++i)
    {
      extension_weights[i] = 1 / (2 * (N + K) + lambda);
    }
  }

  void predict()
  {
    //计算sigma点
    getEffectiveCov(cov);
    computeSigmaPoints(mean, cov, sigma_points);
    for(int i = 0; i < S; ++i)
    {
      sigma_points.row(i) = system.f(sigma_points.row(i));
    }
    const auto& R = process_noise;

    //UT变换
    VectorXt mean_pred = VectorXt::Zero(mean.size(), 1);
    MatrixXt cov_pred = MatrixXt::Zero(cov.rows(), cov.cols());
    for(int i = 0; i < S; ++i)
    {
      mean_pred += weights[i] * sigma_points.row(i);
    }
    for(int i = 0; i < S; ++i)
    {
      VectorXt diff = sigma_points.row(i).transpose() - mean_pred;
      cov_pred += weights[i] * diff * diff.transpose();
    }
    cov_pred += R;
    mean = mean_pred;
    cov = cov_pred;
  }

  void predict(const VectorXt& control) {
    //计算sigma点
    getEffectiveCov(cov);
    computeSigmaPoints(mean, cov, sigma_points);
    for (int i = 0; i < S; i++) 
    {
      sigma_points.row(i) = system.f(sigma_points.row(i), control);
    }
    const auto& R = process_noise;

    //UT变换
    VectorXt mean_pred = VectorXt::Zero(mean.size());
    MatrixXt cov_pred = MatrixXt::Zero(cov.rows(), cov.cols());
    for (int i = 0; i < S; i++) 
    {
      mean_pred += weights[i] * sigma_points.row(i);
    }
    for (int i = 0; i < S; i++) 
    {
      VectorXt diff = sigma_points.row(i).transpose() - mean_pred;
      cov_pred += weights[i] * diff * diff.transpose();
    }
    cov_pred += R;
    mean = mean_pred;
    cov = cov_pred;
  }

  void correct(const VectorXt& measurement)
  {
    VectorXt extension_mean_pred = VectorXt::Zero(N + K, 1);
    MatrixXt extension_cov_pred = MatrixXt::Zero(N + K, N + K);
    extension_mean_pred.topLeftCorner(N, 1) = VectorXt(mean);
    extension_cov_pred.topLeftCorner(N, N) = MatrixXt(cov);
    extension_cov_pred.bottomRightCorner(K, K) = measurement_noise;

    getEffectiveCov(extension_cov_pred);
    computeSigmaPoints(extension_mean_pred, extension_cov_pred, extension_sigma_points);

    //UT变换
    expected_measurements.setZero();
    for (int i = 0; i < extension_sigma_points.rows(); i++) {
      expected_measurements.row(i) = system.h(extension_sigma_points.row(i).transpose().topLeftCorner(N, 1));
      expected_measurements.row(i) += VectorXt(extension_sigma_points.row(i).transpose().bottomRightCorner(K, 1));
    }

    VectorXt expected_measurement_mean = VectorXt::Zero(K);
    for (int i = 0; i < extension_sigma_points.rows(); i++) {
      expected_measurement_mean += extension_weights[i] * expected_measurements.row(i);
    }
    MatrixXt expected_measurement_cov = MatrixXt::Zero(K, K);
    for (int i = 0; i < extension_sigma_points.rows(); i++) {
      VectorXt diff = expected_measurements.row(i).transpose() - expected_measurement_mean;
      expected_measurement_cov += extension_weights[i] * diff * diff.transpose();
    }

    // calculated transformed covariance
    MatrixXt sigma = MatrixXt::Zero(N + K, K);
    for (int i = 0; i < extension_sigma_points.rows(); i++) {
      auto diffA = (extension_sigma_points.row(i).transpose() - extension_mean_pred);
      auto diffB = (expected_measurements.row(i).transpose() - expected_measurement_mean);
      sigma += extension_weights[i] * (diffA * diffB.transpose());
    }
    kalman_gain = sigma * expected_measurement_cov.inverse();
    VectorXt extension_mean = extension_mean_pred + kalman_gain * (measurement - expected_measurement_mean);
    MatrixXt extension_cov = extension_cov_pred - kalman_gain * expected_measurement_cov * kalman_gain.transpose();

    mean = extension_mean.topLeftCorner(N, 1);
    cov = extension_cov.topLeftCorner(N, N);
  }

private :
  void getEffectiveCov(MatrixXt& cov)
  {
    auto min_cov = 1e-9;
    
    Eigen::EigenSolver<MatrixXt> solver(cov);
    MatrixXt D = solver.pseudoEigenvalueMatrix();   
    MatrixXt V = solver.pseudoEigenvectors();       
    for(int i = 0; i < D.rows(); ++i)
    {
      if(D(i, i) < min_cov)
        D(i, i) = min_cov;
    }
    cov = V * D * V.inverse();
  }

  void computeSigmaPoints(const VectorXt& mean, const MatrixXt& cov, MatrixXt& sigma_points)
  {
    auto n = mean.size();

    //分解协方差矩阵
    Eigen::LLT<MatrixXt> llt;
    llt.compute((n + lambda) * cov);
    MatrixXt l = llt.matrixL();

    sigma_points.row(0) = mean;
    for(int i = 0; i < n; ++i)
    {
      sigma_points.row(2 * i + 1) = mean + l.col(i);
      sigma_points.row(2 * i + 1 + 1) = mean - l.col(i);
    }
  }

private :
  const int state_dim;                                //状态向量维度
  const int input_dim;                                //输入向量维度
  const int measurement_dim;                          //观测向量维度
  const int N;
  const int M;
  const int K;
  const int S;                                        //sigma点的数量

public :
  const VectorXt& getMean() const { return mean; }
  const MatrixXt& getCov() const { return cov; }
  const MatrixXt& getSigmaPoints() const { return sigma_points;}  
  const System& getSystem() const { return system; }
  const MatrixXt& getProcessNoiseCov() const { return process_noise; } 
  const MatrixXt& getMeasurementNoiseCov() const { return measurement_noise; }
  const MatrixXt& getKalmanGain() const { return kalman_gain; }
  UnscentedKalmanFilterX& setMean(const VectorXt& m) { mean = m; return *this; }
  UnscentedKalmanFilterX& setCov(const MatrixXt& s) { cov = s; return *this; }
  UnscentedKalmanFilterX& setProcessNoiseCov(const MatrixXt& p) { process_noise = p; return *this; }
  UnscentedKalmanFilterX& setMeasurementNoiseCov(const MatrixXt& m) { measurement_noise = m; return *this; }
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

public :
  std::mt19937 mt;                                    //随机数生成引擎
  std::normal_distribution<T> normal_dist;            //正态分布生成随机数 均值为0 标准差为1
  
  VectorXt mean;                                      //状态向量的估计均值
  MatrixXt cov;                                       //状态向量的协方差矩阵

  System system;                                      //状态转移和观测函数
  MatrixXt process_noise;                             //过程噪声
  MatrixXt measurement_noise;                         //观测噪声
 
  T lambda;                                           //确定sigma点分布

  VectorXt weights;                                   //sigma点的权重
  VectorXt extension_weights;                         //扩展sigma点的权重
  MatrixXt sigma_points;                              //sigma点的矩阵
  MatrixXt extension_sigma_points;                    //扩展sigma点的矩阵
  MatrixXt expected_measurements;                     //预期观测值矩阵
  MatrixXt kalman_gain;                               //卡尔曼增益
};



}


#endif