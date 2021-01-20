#ifndef CONVEX_HULL_H
#define CONVEX_HULL_H

#include "Shape.h"

#include <iostream>
#include <stack>
#include <vector>

namespace utils {

/* class to provide convex hull functionalities */
class ConvexHull {
public:
  /* explicit constructor to get intersection points upon initialization */
  explicit ConvexHull(std::vector<Point>&& intersectionPoints) : intersectionPoints_(std::move(intersectionPoints)) {}

  /* function to get the convex hull points
   * returns the convex hull points if there are at least 3 intersection points and
   * they are not collinear, otherwise an empty std::vector<Point>
   */
  std::vector<Point> getConvexHull();

  /* function to return the area of the convex hull
   * returns the area of the convex hull if the hullPoints_ container is not empty,
   * otherwise zero
   */
  double getArea();

private:
  /* helper function to get the orientation of three points (e.g. clockwise) */
  inline double orientation(const Point& p1, const Point& p2, const Point& p3);

  /* helper function to store the convex hull points internally for future usage */
  void storeHullPoints(const std::vector<Point>& hullPoints);

  /* container to store the convex hull points */
  std::vector<Point> hullPoints_;
  /* container to store the intersection points */
  std::vector<Point> intersectionPoints_;
};

} // namespace utils

#endif // CONVEX_HULL_H
