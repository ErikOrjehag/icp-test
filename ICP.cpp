#include "ICP.hpp"
#include <limits>


Pose icp_align(
    std::vector<Vec2> &source,
    std::vector<Vec2> &targets, 
    Pose pose, int iterations)
{
    Pose acc;

    Vec2 *closest;
    std::vector<const Vec2*> a, b;
    pose.transform(&targets);

    for (int i = 0; i < iterations; i++)
    {
        a.clear();
        b.clear();

        for(std::vector<Vec2>::iterator t = targets.begin(); t != targets.end(); t++)
        {
            closest = icp_closest_point(source, *t);
            if (!closest) continue;
            a.push_back(&(*t));
            b.push_back(closest);
        }

        Pose rel = Pose::relativePose(a, b);
        rel.transform(&targets);
        acc.accumulate(rel);
    }

    return acc;
}

Vec2* icp_closest_point(
    std::vector<Vec2> &points,
    Vec2 &point)
{
    double minDist = std::numeric_limits<float>::infinity();
    Vec2 *closestPoint = NULL;

    for (std::vector<Vec2>::iterator p = points.begin(); p != points.end(); p++)
    {
        double sqrdDist = point.sqrdDistTo(*p);
        if (sqrdDist < minDist)
        {
            minDist = sqrdDist;
            closestPoint = &(*p);
        }
    }

    return closestPoint;
}
