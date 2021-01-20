#define CATCH_CONFIG_MAIN  // This tells Catch2 to provide a main() - only do this in one cpp file

#include "catch.hpp"

#include "ConvexHull.h"
#include "Shape.h"

#include <chrono>
#include <random>


using namespace utils;

TEST_CASE("Creating Point", "[create point]") {
  Point p;

  REQUIRE(p.x == 0.0);
  REQUIRE(p.y == 0.0);
}

TEST_CASE("Creating Point with coordinates", "[create custom point]") {
  Point p(1.1, 2.2);

  REQUIRE(p.x == 1.1);
  REQUIRE(p.y == 2.2);
}

TEST_CASE("Are Points the same", "[same points]") {
  Point p1(1.1, 2.2);
  Point p2(1.1, 2.2);
  Point p3(1.2, 2.3);

  REQUIRE(p1 == p2);
  REQUIRE_FALSE(p1 == p3);
}

TEST_CASE("Is Point1 smaller than Point2", "[smaller point]") {
  Point p1(1.1, 2.2);
  Point p2(1.7, 2.5);

  REQUIRE(p1 < p2);
}

TEST_CASE("Creating Line Segment", "[create line segment]") {
  LineSegment ls;

  ls.begin.x = 0.0;
  ls.begin.y = 0.0;
  ls.end.x = 2.0;
  ls.end.y = 2.0;

  REQUIRE(ls.begin.x == 0.0);
  REQUIRE(ls.begin.y == 0.0);
  REQUIRE(ls.end.x == 2.0);
  REQUIRE(ls.end.y == 2.0);
}

TEST_CASE("Creating Circle", "[create circle]") {
  Circle c;

  c.center.x = 7.0;
  c.center.y = 7.0;
  c.radius = 3;

  REQUIRE(c.center.x == 7.0);
  REQUIRE(c.center.y == 7.0);
  REQUIRE(c.radius == 3);
}

TEST_CASE("Testing no intersection points", "[no intersection]") {
  LineSegment ls;
  Circle c;
  Shape shape;

  ls.begin.x = 11.0;
  ls.begin.y = 11.0;
  ls.end.x = 12.0;
  ls.end.y = 12.0;

  c.center.x = 0.0;
  c.center.y = 0.0;
  c.radius = 2;

  shape.addShape(ls);
  shape.addShape(c);

  auto intersections = shape.getIntersectionPoints();

  CHECK(intersections.size() == 0);
}

TEST_CASE("Testing intersection points", "[intersection]") {
  LineSegment ls;
  Circle c;
  Shape shape;

  ls.begin.x = 1.0;
  ls.begin.y = 0.0;
  ls.end.x = 3.0;
  ls.end.y = 0.0;

  c.center.x = 0.0;
  c.center.y = 0.0;
  c.radius = 2;

  shape.addShape(ls);
  shape.addShape(c);

  auto intersections = shape.getIntersectionPoints();

  CHECK(intersections.size() == 1);
}

TEST_CASE("Empty Convex Hull", "[empty convex hull]") {
  LineSegment ls;
  Circle c;
  Shape shape;

  ls.begin.x = 1.0;
  ls.begin.y = 0.0;
  ls.end.x = 3.0;
  ls.end.y = 0.0;

  c.center.x = 7.0;
  c.center.y = 7.0;
  c.radius = 1;

  shape.addShape(ls);
  shape.addShape(c);

  auto intersections = shape.getIntersectionPoints();

  ConvexHull hull(std::move(intersections));
  auto hullPoints = hull.getConvexHull();

  CHECK(hullPoints.size() == 0);
}

TEST_CASE("Not empty Convex Hull", "[convex hull]") {
  LineSegment ls;
  Circle c1;
  Circle c2;
  Shape shape;

  ls.begin.x = -3.0;
  ls.begin.y = -3.0;
  ls.end.x = -1.0;
  ls.end.y = 2.0;

  c1.center.x = 0.0;
  c1.center.y = 0.0;
  c1.radius = 5;

  c2.center.x = 3.0;
  c2.center.y = 2.0;
  c2.radius = 7;

  shape.addShape(ls);
  shape.addShape(c1);
  shape.addShape(c2);

  auto intersections = shape.getIntersectionPoints();

  ConvexHull hull(std::move(intersections));
  auto hullPoints = hull.getConvexHull();

  CHECK(hullPoints.size() > 0);
}

TEST_CASE("Zero area if points are collinear" , "[zero area]") {
  LineSegment ls;
  Circle c1;
  Circle c2;
  Circle c3;
  Shape shape;

  ls.begin.x = 1.0;
  ls.begin.y = 0.0;
  ls.end.x = 4.0;
  ls.end.y = 0.0;

  c1.center.x = 0.0;
  c1.center.y = 0.0;
  c1.radius = 2;

  c2.center.x = 0.0;
  c2.center.y = 0.0;
  c2.radius = 3;

  c3.center.x = 0.0;
  c3.center.y = 0.0;
  c3.radius = 4;

  shape.addShape(ls);
  shape.addShape(c1);
  shape.addShape(c2);
  shape.addShape(c3);

  auto intersections = shape.getIntersectionPoints();

  ConvexHull hull(std::move(intersections));
  auto hullPoints = hull.getConvexHull();

  REQUIRE(hullPoints.size() == 0);
  REQUIRE(hull.getArea() == 0.0);
}

TEST_CASE("Measure execution time based on example input", "[execution time example input]") {
  LineSegment ls1;
  LineSegment ls2;
  LineSegment ls3;
  LineSegment ls4;
  Circle c1;
  Circle c2;
  Circle c3;
  Circle c4;
  Shape shape;

  ls1.begin.x = -2.0;
  ls1.begin.y = 1.0;
  ls1.end.x = 12.0;
  ls1.end.y = 6.0;

  ls2.begin.x = -4.0;
  ls2.begin.y = 9.0;
  ls2.end.x = 8.0;
  ls2.end.y = -4.0;

  ls3.begin.x = 5.0;
  ls3.begin.y = 2.0;
  ls3.end.x = 7.0;
  ls3.end.y = 9.0;

  ls4.begin.x = -3.0;
  ls4.begin.y = -3.0;
  ls4.end.x = -1.0;
  ls4.end.y = 2.0;

  c1.center.x = 0.0;
  c1.center.y = 0.0;
  c1.radius = 5;

  c2.center.x = 3.0;
  c2.center.y = 2.0;
  c2.radius = 7;

  c3.center.x = 3.0;
  c3.center.y = 10.0;
  c3.radius = 1;

  c4.center.x = 8.0;
  c4.center.y = 2.0;
  c4.radius = 2;

  auto start = std::chrono::high_resolution_clock::now();

  shape.addShape(ls1);
  shape.addShape(ls2);
  shape.addShape(ls3);
  shape.addShape(ls4);
  shape.addShape(c1);
  shape.addShape(c2);
  shape.addShape(c3);
  shape.addShape(c4);

  auto intersections = shape.getIntersectionPoints();
  size_t intersectionSize = intersections.size();

  ConvexHull hull(std::move(intersections));

  auto hullPoints = hull.getConvexHull();
  size_t hullSize = hullPoints.size();

  double area = hull.getArea();

  auto end = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
  std::cout << "[*****************************************************]" << std::endl;
  std::cout << std::fixed << std::setprecision(4) <<
               "[ Time elapsed during execution [example input]: " << duration << " ms ]" << std::endl;
  std::cout << "[*****************************************************]" << std::endl;

  CHECK(intersectionSize == 15u);
  CHECK(hullSize == 9u);
  CHECK(area <= 140.0776);
}

TEST_CASE("Measure execution time based on random input", "[execution time random input]") {
  std::random_device rd;
  std::mt19937 rng{rd()};
  std::uniform_int_distribution<int> uid(-10000, 10000);
  Shape shape;

  /* generating 1000 shapes (500 LineSegments & 500 Circles) */
  for (size_t i = 0u; i < 500u; ++i) {
    double x1 = uid(rng);
    double y1 = uid(rng);
    double x2 = uid(rng);
    double y2 = uid(rng);

    if (x1 > x2) {
      std::swap(x1, x2);
    }

    if (y1 > y2) {
      std::swap(y1, y2);
    }

    LineSegment ls;
    ls.begin.x = x1;
    ls.begin.y = y1;
    ls.end.x = x2;
    ls.end.y = y2;
    shape.addShape(ls);
  }

  for (size_t i = 0u; i < 500u; ++i) {
    double x = uid(rng);
    double y = uid(rng);
    int32_t r = uid(rng) / 100;

    Circle c;
    c.center.x = x;
    c.center.y = y;
    c.radius = r;
    shape.addShape(c);
  }

  auto start = std::chrono::high_resolution_clock::now();

  auto intersections = shape.getIntersectionPoints();

  ConvexHull hull(std::move(intersections));

  auto hullPoints = hull.getConvexHull();

  (void)hull.getArea();

  auto end = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
  std::cout << "[***********************************************************]" << std::endl;
  std::cout << std::fixed << std::setprecision(4) <<
               "[ Time elapsed during execution [1000 random input]: " << duration << " ms ]" << std::endl;
  std::cout << "[***********************************************************]" << std::endl;
}
