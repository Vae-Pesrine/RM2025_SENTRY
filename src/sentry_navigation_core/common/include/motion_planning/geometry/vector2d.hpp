#ifndef COMMON_MOTION_PLANNING_GEOMETRY_Vector2d_HPP_
#define COMMON_MOTION_PLANNING_GEOMETRY_Vector2d_HPP_

#pragma once

#include <cmath>
#include <cstring>
namespace common
{
namespace geometry
{
constexpr double kMathEpsilon = 1e-10;

class Vector2d
{
public:
    constexpr Vector2d(const double x, double y) noexcept: x_(x), y_(y)
    { 
    }

    constexpr Vector2d() noexcept: Vector2d(0, 0)
    {
    }

    // 创建一个与正x半轴成一定角度的向量
    static Vector2d createUnitVector2d(const double angle);

    double x() const
    {
        return x_;
    }

    double y() const
    {
        return y_;
    }

    void setX(double x)
    {
        x_ = x;
    }

    void setY(double y)
    {
        y_ = y;
    }

    // 获取向量的长度，即 sqrt(x^2 + y^2)
    double length() const;

    // 获取向量的长度的平方，即 x^2 + y^2  等价于 length() * length()
    double lengthSquare() const;

    // 获取向量与正x半轴的夹角，返回值在 [-pi, pi] 之间
    double angle() const;

    // 获取单位向量
    void normalize();

    // 获取向量与另一个向量的距离，即 sqrt((x1 - x2)^2 + (y1 - y2)^2)
    double distanceTo(const Vector2d& other) const;

    // 获取向量与另一个向量的距离的平方
    double distanceSquareTo(const Vector2d& other) const;

    // 返回两个向量的叉积
    double crossProd(const Vector2d& other) const;

    // 获取两个向量的内积，即 x1 * y2 + x2 * y1
    double innerProd(const Vector2d& other) const;

    // 旋转向量
    Vector2d rotate(const double angle) const;

    // 旋转向量本身
    void selfRotate(const double angle);

    // 两个向量相加
    Vector2d operator+(const Vector2d& other) const;

    // 两个向量相减
    Vector2d operator-(const Vector2d& other) const;

    // 将向量与标量相乘
    Vector2d operator*(const double ratio) const;

    // 将向量与标量相除
    Vector2d operator/(const double ratio) const;

    // 将另一个向量加到当前向量
    Vector2d& operator+=(const Vector2d& other);

    // 从当前向量减去另一个向量
    Vector2d& operator-=(const Vector2d& other);

    // 将当前向量与标量相乘
    Vector2d& operator*=(const double ratio);

    // 将当前向量与标量相除
    Vector2d& operator/=(const double ratio);

    // 判断两个向量是否相等
    bool operator==(const Vector2d& other) const;

protected:
    double x_ = 0.0;
    double y_ = 0.0;
};

//将给定的向量与标量相乘
Vector2d operator*(const double ratio, const Vector2d& vec);

} // namespace geometry
} // namespace common


#endif