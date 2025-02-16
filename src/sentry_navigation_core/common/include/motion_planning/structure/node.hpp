#ifndef MOTION_PLANNING_STRUCTURE_NODE_HPP_
#define MOTION_PLANNING_STRUCTURE_NODE_HPP_

#include <cmath>
#include <array>
#include <vector>
#include <map>

namespace common
{
namespace structure
{
/**
 * @brief 基础节点类
 */
template <typename T>
class Node
{
public:
    /**
     * @brief Node 类的构造函数
     * @param x   x 值
     * @param y   y 值
     * @param g   g 值，到达该节点的成本
     * @param h   h 值，该节点的启发式成本
     * @param id  节点的 ID
     * @param pid 父节点的 ID
     */
    Node(T x = 0, T y = 0, double g = 0.0, double h = 0.0, int id = 0, int pid = 0)
        : x_(x), y_(y), g_(g), h_(h), id_(id), pid_(pid) {};

    Node (const Node& n): x_(n.x()), y_(n.y()), g_(n.g()), h_(n.h()), id_(n.id()), pid_(n.pid()) {};


    /**
      * @brief 获取节点的属性
      */
    T x() const
    {
        return x_;
    }

    T y() const
    {
        return y_;
    }

    double g() const
    {
        return g_;
    }

    double h() const
    {
        return h_;
    }

    int id() const
    {
        return id_;
    }

    int pid() const
    {
        return pid_;
    }

    /**
      * @brief 设置节点的属性
      */
    void set_x(T x)
    {
        x_ = x;
    }

    void set_y(T y)
    {
        y_ = y;
    }

    void set_g(double g)
    {
        g_ = g;
    }

    void set_h(double h)
    {
        h_ = h;
    }

    void set_id(int id)
    {
        id_ = id;
    }

    void set_pid(int pid)
    {
        pid_ = pid;
    }

    /**
     * @brief 重载 + 操作符
     * @param n 另一个节点
     * @return 当前节点和输入节点 n 的值相加后的节点
     */
    Node operator+(const Node& n) const
    {
        return Node(x_ + n.x(), y_ + n.y());
    }

    /**
     * @brief 重载 - 操作符
     * @param n 另一个节点
     * @return 当前节点和输入节点 n 的值相减后的节点
     */
    Node operator-(const Node& n) const
    {
        return Node(x_ - n.x(), y_ - n.y());
    }

    /**
     * @brief 重载 == 操作符
     * @param n 另一个节点
     * @return 如果当前节点等于节点 n，则返回 true，否则返回 false
     */
    bool operator==(const Node& n) const
    {
        return (x_ == n.x()) && (y_ == n.y());
    }

    /**
     * @brief 重载 != 操作符
     * @param n 另一个节点
     * @return 如果当前节点不等于节点 n，则返回 true，否则返回 false
     */
    bool operator!=(const Node& n) const
    {
        return!(*this == n);
    }

    /**
     * @brief 比较两个节点的代价
     *        用于多种算法和类中
     */
    struct compare_cost
    {
        /**
         * @brief 比较两个节点的代价
         * @param n1 一个节点
         * @param n2 另一个节点
         * @return 如果到达 n1 的代价大于 n2，则返回 true，否则返回 false
         */
        bool operator()(const Node& a, const Node& b) const
        {
            return (a.g() + a.h() > b.g() + b.h()) || ((a.g() + a.h() == b.g() + b.h()) && (a.h() > b.h()));
        };
    };


    /**
     * @brief 比较两个节点的坐标
     *        用于多种算法和类中
     */
    struct compare_corrinates
    {
        /**
         * @brief 比较两个节点的坐标
         * @param n1 一个节点
         * @param n2 另一个节点
         * @return 如果 n1 的坐标等于 n2，则返回 true，否则返回 false
         */
        bool operator()(const Node& a, const Node& b) const
        {
            return (a.x() == b.x()) && (a.y() == b.y());
        };
    };

protected:
    T x_, y_;
    double g_, h_;
    int id_, pid_;
};
} // namespace structure
} // namespace common

#endif