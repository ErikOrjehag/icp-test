#pragma once

#include "Vec2.hpp"
#include <vector>
#include <iostream>

class Pose {
public:
    Vec2 pos;
    double angle = 0;

    void transform(std::vector<Vec2> *points);
    void accumulate(const Pose &pose);
    static Pose relativePose(std::vector<const Vec2*> &a, std::vector<const Vec2*> &b);
};

std::ostream &operator<<(std::ostream &os, const Pose &p);