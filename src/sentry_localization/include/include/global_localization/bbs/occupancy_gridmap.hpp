#ifndef OCCUPANCY_GRIDMAP_HPP
#define OCCUPANCY_GRIDMAP_HPP

#include <vector>
#include <memory>
#include <Eigen/Core>
#include <nav_msgs/OccupancyGrid.h>
#include <opencv2/opencv.hpp>

namespace sentry_localization
{
class OccupancyGridMap
{
public:
    using Ptr = std::shared_ptr<OccupancyGridMap>;
    
    //初始化地图
    OccupancyGridMap(int width, int height, double resolution)
    {
        this->resolution = resolution;
        values = cv::Mat1f(height, width, 0.0f);
    }

    OccupancyGridMap(double resolution, const cv::Mat1f& values)
    {
        this->resolution = resolution;
        this->values = values;
    }

    double grid_resolution() const{
        return resolution;
    }
    int width() const{
        return values.cols;
    }
    int height() const{
        return values.rows;
    }
    //返回地图数据指针
    const float* data() const{
        return reinterpret_cast<const float*>(values.data);
    }

public:
    //将点云数据插入地图
    void insert_points(const std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>>& points, int min_points_per_grid)
    {
        for(const auto& pt : points){
            auto loc = grid_loc(pt);
            if(in_map(loc)){
                value(loc) += 1;
            }
        }

        values /= min_points_per_grid;
    }

    double calc_score(const std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>>& points) const{
        double sum = 0.0;
        for(const auto& pt : points){
            auto loc = grid_loc(pt);
            loc.x() = std::max<int>(0, std::min<int>(values.cols - 1, loc.x()));
            loc.y() = std::max<int>(0, std::min<int>(values.rows - 1, loc.y()));
            sum += value(loc);
        }
        return sum;
    }

    //地图上采样，分辨率更高
    OccupancyGridMap::Ptr pyramid_up() const{
        cv::Mat1f small_map(values.rows / 2, values.cols / 2);
        for(int i = 0; i < values.rows / 2; ++i){
            for(int j = 0; j < values.cols / 2; ++j){
                float x = values.at<float>(2 * i, 2 * j);
                x = std::max(x, values.at<float>(2 * i + 1, 2 * j));
                x = std::max(x, values.at<float>(2 * i, 2 * j + 1));
                x = std::max(x, values.at<float>(2 * i + 1, 2 * j + 1));
                small_map.at<float>(i, j) = x;
            }
        }
        return std::make_shared<OccupancyGridMap>(resolution * 2.0, small_map);
    }

    nav_msgs::OccupancyGridConstPtr to_rosmsg() const{
        nav_msgs::OccupancyGridPtr msg(new nav_msgs::OccupancyGrid);
        msg->header.frame_id = "map";
        msg->header.stamp = ros::Time(0);
        msg->data.resize(values.cols * values.rows);

        std::transform(values.begin(), values.end(), msg->data.begin(), [=](auto x){
            double x_ = x * 100.0;
            return std::max(0.0, std::min(100.0, x_));
        });

        msg->info.map_load_time = ros::Time::now();
        msg->info.width = values.cols;
        msg->info.height = values.rows;
        msg->info.resolution = resolution;
        msg->info.origin.position.x = -resolution * values.cols / 2;
        msg->info.origin.position.y = -resolution * values.rows / 2;
        msg->info.origin.position.z = 0.0;
        return msg;
    }

private:
    //检查点是否在地图中
    bool in_map(const Eigen::Vector2i& pix)  const{
        bool left_bound = (pix.array() >= Eigen::Array2i::Zero()).all();
        bool right_bound = (pix.array() < Eigen::Array2i(values.cols, values.rows)).all();
        return left_bound && right_bound;
    }

    float value(const Eigen::Vector2i& loc) const{
        return values.at<float>(loc.y(), loc.x());
    }
    float& value(const Eigen::Vector2i& loc){
        return values.at<float>(loc.y(), loc.x());
    }

    //将点转化为地图中的栅格位置
    Eigen::Vector2i grid_loc(const Eigen::Vector2f& pt) const{
        Eigen::Vector2i loc = (pt / resolution).array().floor().cast<int>();
        Eigen::Vector2i offset = Eigen::Vector2i(values.cols / 2, values.rows / 2);
        return loc + offset;
    }

private:
    double resolution;
    cv::Mat1f values;
};
}



#endif