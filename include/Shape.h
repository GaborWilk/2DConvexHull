#ifndef SHAPE_H
#define SHAPE_H

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <optional>
#include <tuple>
#include <vector>

namespace utils {

/* helper function to calculate the distance between two Points */
struct Point;
double distanceBetweenPoints(const Point& lhs, const Point& rhs);

/* structure to store x and y coordinates */
struct Point {
  double x{0.0};
  double y{0.0};

  Point() {}
  Point(double X, double Y) : x{X}, y{Y} {}

  bool operator==(const Point& p) const;
  bool operator<(const Point& p) const;
};

/* structure to store Line Segments */
struct LineSegment {
  Point begin;
  Point end;

  LineSegment() {}
  LineSegment(const Point& p1, const Point& p2) : begin{p1}, end{p2} {}
};

/* structure to store Circles */
struct Circle {
  Point center;
  double radius;

  Circle() {}
  Circle(const Point& p, double rad) : center{p}, radius{rad} {}
};

/* class to store Line Segments and Circles in a general way */
class Shape {
public:
  Shape() = default;

  /* functions to add Line Segment/Circle to the internal storage */
  void addShape(const LineSegment& ls);
  void addShape(const Circle& c);

  /* function to calculate intersection points
   * returns the unique intersection points in ascending order based on the x coordinate,
   * or and empty std::vector<Point> if there are no intersections at all
   */
  std::vector<Point> getIntersectionPoints();

private:
  /* helper functions to get intersection points based on the shape types */
  std::optional<Point> getIntersectionPoint(const LineSegment& lhs, const LineSegment& rhs);
  std::vector<Point> getIntersectionPoint(const LineSegment& ls, const Circle& c);
  std::vector<Point> getIntersectionPoint(const Circle& lhs, const Circle& rhs);

  /* heterogeneus container to store shapes in a general way */
  std::tuple<std::vector<LineSegment>, std::vector<Circle>> shapes_;
};

} // namespace utils

#endif // SHAPE_H
