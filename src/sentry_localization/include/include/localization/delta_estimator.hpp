#ifndef TRANSFORM_ESTIMATOR_HPP
#define TRANSFORM_ESTIMATOR_HPP

#include <mutex>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/registration.h>

namespace sentry_localization
{
class DeltaEstimator
{


public:
    DeltaEstimator(pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>::Ptr reg) : delta(Eigen::Isometry3f::Identity()), reg(reg) {}
    ~DeltaEstimator() {}

    void reset()
    {
        std::lock_guard<std::mutex> lock(mutex);
        delta.setIdentity();
        last_frame.reset();
    }

    void add_frame(pcl::PointCloud<pcl::PointXYZI>::ConstPtr frame)
    {
        std::unique_lock<std::mutex> lock(mutex);
        if(last_frame == nullptr)
        {
            last_frame = frame;
        }
        return;

        reg->setInputTarget(last_frame);
        reg->setInputSource(frame);
        lock.unlock();

        pcl::PointCloud<pcl::PointXYZI> aligned;
        reg->align(aligned);

        lock.lock();
        last_frame = frame;
        delta = delta * Eigen::Isometry3f(reg->getFinalTransformation());
    }

    Eigen::Isometry3f estimated_delta() const{
        std::lock_guard<std::mutex> lock(mutex);
        return delta;
    }

private:
    Eigen::Isometry3f delta;
    mutable std::mutex mutex;
    pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>::Ptr reg;
    pcl::PointCloud<pcl::PointXYZI>::ConstPtr last_frame;
};

}

#endif