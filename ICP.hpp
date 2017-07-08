#pragma once

#include "Pose.hpp"
#include "Vec2.hpp"

Pose icp_align(
    std::vector<Vec2> &source, 
    std::vector<Vec2> &target, 
    Pose pose, int iterations);

Vec2* icp_closest_point(
    std::vector<Vec2> &points,
    Vec2 &point);