//
// Created by bacon on 22-10-2.
//

#ifndef MYTURTLE_POINT_H
#define MYTURTLE_POINT_H
#include <cassert>
#include <cstring>
#include <memory>
#include <vector>
// #include "CConfig.h
#include "CMatStruct.h"
#include "math.h"
#include "stdint.h"
class Pose2dT;
class Point2dT {  // todo: move to math someday
 public:
  float x, y;
  Point2dT() : x(0), y(0){};
  Point2dT(float x, float y) : x(x), y(y){};
  explicit Point2dT(const CPoint2D& p) : x(p.x), y(p.y){};
  Point2dT operator-(const Point2dT& p) const { return {x - p.x, y - p.y}; }
  Point2dT operator+(const Point2dT& p) const { return {x + p.x, y + p.y}; }
  float len() { return sqrtf(x * x + y * y); };
  void setPoint(double x, double y) {
    this->x = x;
    this->y = y;
  }
  //    explicit Point2dT(const CPose2D& p) :x(p.x), y(p.y){};
};

class Point2iT {  // todo: move to math someday
 public:
  int x, y;
  Point2iT() : x(0), y(0){};
  Point2iT(int x, int y) : x(x), y(y){};
  Point2iT operator-(const Point2iT& p) const { return {x - p.x, y - p.y}; }
  Point2iT operator+(const Point2iT& p) const { return {x + p.x, y + p.y}; }
};

class Pose2dT {  // todo: move to math someday
 public:
  float x, y, th;
  Pose2dT() : x(0), y(0), th(0){};
  Pose2dT(float x, float y, float th) : x(x), y(y), th(th){};
  explicit Pose2dT(const CPose2D& p) : x(p.x), y(p.y), th(p.th){};

  Pose2dT inverse() const {
    float c = cosf(th);
    float s = sinf(th);
    return {-c * x - s * y, s * x - c * y, -th};
  }

  Pose2dT operator*(const Pose2dT& p) const {
    float c = cosf(th);
    float s = sinf(th);
    return {x + c * p.x - s * p.y, y + s * p.x + c * p.y, th + p.th};
  }

  Pose2dT operator+(const Point2dT& p) const { return {x + p.x, y + p.y, th}; }
  Pose2dT operator-(const Point2dT& p) const { return {x - p.x, y - p.y, th}; }

  Point2dT operator*(const Point2dT& p) const {
    float c = cosf(th);
    float s = sinf(th);
    return {x + c * p.x - s * p.y, y + s * p.x + c * p.y};
  }

  //    Point2dT operator = (const Pose2dT& p )  //丑
  //    {
  //        return { p.x , p.y};
  //    }

  Pose2dT& operator=(const CPose2D& p) {
    this->x = p.x;
    this->y = p.y;
    this->th = p.th;
    return *this;
  }
};

#endif  // MYTURTLE_POINT_H
