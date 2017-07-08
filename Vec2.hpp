#pragma once

class Vec2 {

public:

    double x;
    double y;

    Vec2();
    Vec2(double x, double y);

    double sqrdDistTo(const Vec2 &other);

};