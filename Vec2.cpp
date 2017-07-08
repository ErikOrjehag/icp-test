#include "Vec2.hpp"
#include <cmath>

Vec2::Vec2() {

}

Vec2::Vec2(double x, double y) : x(x), y(y) {}

double Vec2::sqrdDistTo(const Vec2 &other)
{
    return std::pow(x - other.x, 2) + std::pow(y - other.y, 2);
}