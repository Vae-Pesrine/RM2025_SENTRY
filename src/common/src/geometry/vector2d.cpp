#include "motion_planning/util/log.hpp"
#include "motion_planning/geometry/vector2d.hpp"

namespace common
{
namespace geometry
{
// 创建一个与正x半轴成一定角度的向量
Vector2d Vector2d::createUnitVector2d(const double angle)
{
    return Vector2d(cos(angle), sin(angle));
}

// 获取向量的长度，即 sqrt(x^2 + y^2)
double Vector2d::length() const
{
    return std::hypot(x_, y_);
}

// 获取向量的长度的平方，即 x^2 + y^2  等价于 length() * length()
double Vector2d::lengthSquare() const
{
    return x_ * x_ + y_ * y_;
}

// 获取向量与正x半轴的夹角，返回值在 [-pi, pi] 之间
double Vector2d::angle() const
{
    return std::atan2(y_, x_);
}

// 获取单位向量
void Vector2d::normalize()
{
    const double l = length();
    if(l > kMathEpsilon){
        x_ /= l;
        y_ /= l;
    }
}

// 获取向量与另一个向量的距离，即 sqrt((x1 - x2)^2 + (y1 - y2)^2)
double Vector2d::distanceTo(const Vector2d& other) const
{
    return std::hypot(x_ - other.x_, y_ - other.y_);
}

// 返回两个向量的叉积
double Vector2d::crossProd(const Vector2d& other) const
{
    return x_ * other.y() - y_ * other.x();
}

// 获取两个向量的内积，即 x1 * y2 + x2 * y1
double Vector2d::innerProd(const Vector2d& other) const
{
    return x_ * other.x() + y_ * other.y();
}

// 旋转向量
Vector2d Vector2d::rotate(const double angle) const
{
    return Vector2d(x_ * cos(angle) - y_ * sin(angle), x_ * sin(angle) + y_ * cos(angle));
}

// 旋转向量本身
void Vector2d::selfRotate(const double angle) 
{
    double tmp_x = x_;
    x_ = x_ * cos(angle) - y_ * sin(angle);
    y_ = tmp_x * sin(angle) + y_ * cos(angle);
}

// 两个向量相加
Vector2d Vector2d::operator+(const Vector2d& other) const
{
  return Vector2d(x_ + other.x(), y_ + other.y());
}

// 两个向量相减
Vector2d Vector2d::operator-(const Vector2d& other) const
{
    return Vector2d(x_ - other.x(), y_ - other.y());
}

// 将向量与标量相乘
Vector2d Vector2d::operator*(const double ratio) const
{
    return Vector2d(x_ * ratio, y_ * ratio);
}

// 将向量与标量相除
Vector2d Vector2d::operator/(const double ratio) const
{
    CHECK_GT(std::abs(ratio), kMathEpsilon);
    return Vector2d(x_ / ratio, y_ / ratio);
}

// 将另一个向量加到当前向量
Vector2d& Vector2d::operator+=(const Vector2d& other)
{
    x_ += other.x();
    y_ += other.y();
    return *this;
}

// 从当前向量减去另一个向量
Vector2d& Vector2d::operator-=(const Vector2d& other)
{
    x_ -= other.x();
    y_ -= other.y();
    return *this;   
}

// 将当前向量与标量相乘
Vector2d& Vector2d::operator*=(const double ratio)
{
    x_ *= ratio;
    y_ *= ratio;
    return *this;
}

// 将当前向量与标量相除
Vector2d& Vector2d::operator/=(const double ratio)
{
    CHECK_GT(std::abs(ratio), kMathEpsilon);
    x_ /= ratio;
    y_ /= ratio;
    return *this;
}

// 判断两个向量是否相等
bool Vector2d::operator==(const Vector2d& other) const
{
    return (std::abs(x_ - other.x()) < kMathEpsilon && std::abs(y_ - other.y()) < kMathEpsilon);
}

//将给定的向量与标量相乘
Vector2d operator*(const double ratio, const Vector2d& vec)
{
    return vec * ratio;
}

} // namespace geometry
} // namespace common





























