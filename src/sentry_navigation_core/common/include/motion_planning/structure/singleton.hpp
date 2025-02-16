#ifndef MOTION_PLANNING_STRUCTURE_SINGLETON_HPP_
#define MOTION_PLANNING_STRUCTURE_SINGLETON_HPP_

namespace common
{
namespace structure
{
template <typename TSingleton>
class Singleton
{
public:
    using TsingletonPtr = std::unique_ptr<TSingleton>;

private:
    Singleton() = default;
    virtual ~Singleton() = default;
    Singleton(const Singleton&) = delete;
    Singleton(const Singleton&&) = delete;
    Singleton& operator=(const Singleton&) = delete;

public:
    static TsingletonPtr& Instance()
    {
        static TsingletonPtr instance = std::make_unique<TSingleton>();
        return instance;
    }

};
} // namespace structure
} // namespace common

#endif