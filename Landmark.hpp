
struct Landmark {
  Landmark(Vec2 pos) : pos(pos), timesSeen(0) {};
  Vec2 pos;
  int timesSeen;
};
