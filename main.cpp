#include <iostream>
#include <cmath>

#include "xv11lidar/xv11lidar.h"
#include <SFML/Graphics.hpp>
#include "TransformStack.hpp"
#include "Vec2.hpp"
#include "Pose.hpp"
#include "ICP.hpp"

using namespace std;

const double pi = 3.1415926535897;
const int WIDTH = 800;
const int HEIGHT = 800;

int main() {

  std::vector<Vec2> map;

  const int NUM_READINGS = 90; // 90 frames/revolution

  struct xv11lidar *lidar = xv11lidar_init("/dev/ttyUSB0", NUM_READINGS, 10);

  if (lidar == NULL) {
    std::cout << "LIDAR init error" << std::endl;
    return 1;
  }

  struct xv11lidar_frame frames[NUM_READINGS];

  sf::RenderWindow window(sf::VideoMode(WIDTH, HEIGHT), "SFML window");
  TransformStack ts;
  sf::CircleShape pixel(5);
  pixel.setFillColor(sf::Color::Green);
  /*sf::VertexArray pixel(sf::PrimitiveType::Lines, 2);
  pixel[0] = sf::Vertex(sf::Vector2f(0, 0), sf::Color::Green);
  pixel[1] = sf::Vertex(sf::Vector2f(1000, 0), sf::Color::Green);*/
  ts.translate(WIDTH/2, HEIGHT/2);

  int itr = 0;

  while (window.isOpen()) {

    sf::Event event;
    while (window.pollEvent(event)) {
        if (event.type == sf::Event::Closed) {
          window.close();
        }
    }

    std::vector<Vec2> sweep;

    window.clear();

    int status = xv11lidar_read(lidar, frames);

    if (status == XV11LIDAR_SUCCESS) {
      for (auto frame : frames) {

        for (int i = 0; i < 4; i++) {

          auto reading = frame.readings[i];
          int angle = (frame.index - 0xA0) * 4 + i;

          /*std::cout
            << reading.strength_warning << " : "
            << reading.invalid_data << " : "
            << angle << " : "
            << reading.signal_strength << " : "
            << reading.distance << std::endl;*/

          if (!reading.invalid_data && !reading.strength_warning) {
            float x = reading.distance * cos(angle * pi / 180);
            float y = reading.distance * sin(angle * pi / 180);
            sweep.push_back(Vec2(x, y));
          }
        }
      }
      std::cout << "Speed: " << (frames[0].speed / 64.0) << std::endl;
    } else {
      std::cout << "Error" << std::endl;
    }

    /*Pose icp = icp_align(sweep, map, Pose(), 5);

    for (Vec2 point : sweep) {
      bool foundMatch = false;
      for (std::vector<Vec2>::iterator p = map.begin(); p != map.end(); p++)
      {
        if (point.sqrdDistTo(*p) < pow(25, 2))
        {
          foundMatch = true;
          break;
        }
      }
      if (!foundMatch) {
        map.push_back(point);
      }
    }*/

    ts.push();
    ts.scale(0.3);

    /*pixel.setFillColor(sf::Color::White);
    for (Vec2 point : map) {
      ts.push();
      ts.translate(point.x, point.y);
      window.draw(pixel, ts);
      ts.pop();
    }*/

    pixel.setFillColor(sf::Color::Green);
    for (Vec2 point : sweep) {
      ts.push();
      ts.translate(point.x, point.y);
      window.draw(pixel, ts);
      ts.pop();
    }

    ts.pop();


    /*
    std::vector<Vec2> points;
    points.push_back(Vec2(-50, -50));
    points.push_back(Vec2(-50, 50));
    points.push_back(Vec2(50, 50));
    points.push_back(Vec2(50, -50));
    std::vector<Vec2> points2;
    points2.push_back(Vec2(-50, -50));
    points2.push_back(Vec2(-50, 50));
    points2.push_back(Vec2(50, 50));
    points2.push_back(Vec2(50, -50));
    sf::CircleShape pointShape(4);

    Pose pose;
    pose.pos.x = 30;
    pose.pos.y = 30;
    pose.angle = 0.3;
    //pose.accumulate(pose);
    pose.transform(&points);

    Pose icp = icp_align(points, points2, Pose(), itr++);

    ts.push();
    ts.translate(-2, -2);
    pointShape.setFillColor(sf::Color::Red);
    for (Vec2 point : points) {
      pointShape.setPosition(sf::Vector2f(point.x, point.y));
      window.draw(pointShape, ts);
    }
    pointShape.setFillColor(sf::Color::Green);
    for (Vec2 point : points2) {
      pointShape.setPosition(sf::Vector2f(point.x, point.y));
      window.draw(pointShape, ts);
    }
    ts.pop();
    */

    sf::VertexArray origo(sf::PrimitiveType::Lines, 4);
    origo[0] = sf::Vertex(sf::Vector2f(-1, 0), sf::Color::White);
    origo[1] = sf::Vertex(sf::Vector2f(1, 0), sf::Color::White);
    origo[2] = sf::Vertex(sf::Vector2f(0, -1), sf::Color::White);
    origo[3] = sf::Vertex(sf::Vector2f(0, 1), sf::Color::White);
    ts.push();
    ts.scale(10);
    window.draw(origo, ts);
    ts.pop();

    window.display();
  }

  xv11lidar_close(lidar);
}
