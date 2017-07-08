#include "vector"

class KDTreeVec2 {

public:

    KDTreeVec2();
    KDTreeVec2(std::vector<Vec2> *list);
    KDTreeVec2(std::vector<Vec2> *orderedByX, std::vector<Vec2> *orderedByY, int depth);

private:
    KDTreeVec2 *lt; // Less than
    KDTreeVec2 *gte; // Greater than or equal
    std::vector<Vec2> leafs; // Values

};