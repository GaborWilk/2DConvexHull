# 2D Convex Hull

This tool has two purposes:
1) calculate intersection points of line segments and circles
2) calculate convex hull and its area based on the intersection points

The tool uses 2D Cartesian coordinate system. It reads from the standard input and writes to the standard output.
The implementation is based on the C++17 standard only (CMake 3.8 is required).


## Usage

```cpp
#include "ConvexHull.h"
#include "Shape.h"

using namespace utils;

/* initialize shape container */
Shape shape;

/* read number of shapes from stdin */
/* line segment */
LineSegment ls;
ls.begin.x = 0;
ls.begin.y = 0;
ls.end.x = 3;
ls.end.y = 0;

/* circle */
Circle c;
c.center.x = 0;
c.center.y = 0;
c.radius = 2;

/* add shapes to the shape container */
shpe.addShape(ls);
shpe.addShape(c);

/* get intersection points */
auto intersectionPoints = shape.getIntersectionPoints();

/* number of intersection points */
std::cout << intersectionPoints.size() << std::endl;

/* list intersection points */
if (!intersectionPoints.empty()) {
  for (const auto& it : intersectionPoints) {
std::cout << std::fixed << std::setprecision(4) << it.x << " " << it.y << std::endl;
  }
}

/* initialize convex hull object */
ConvexHull hull(std::move(intersectionPoints));

/* get convex hull points */
auto hullPoints = hull.getConvexHull();

/* number of convex hull points */
std::cout << hullPoints.size() << std::endl;

/* list convex hull points */
if(!hullPoints.empty()) {
  for (const auto& it : hullPoints) {
    std::cout << std::fixed << std::setprecision(4) << it.x << " " << it.y << std::endl;
  }
}

/* calculate convex hull area */
std::cout << std::fixed << std::setprecision(4) << hull.getArea() << std::endl;

```

For more details see the main.cpp file under the src directory.


## Test

I used a 3rd party library ([Catch2](https://github.com/catchorg/Catch2)) for Unit Testing purpose only.

Catch2 is a single-header file to test the tool's functionalities. Because of the large size of the file the compilation time can be above average, but it does not functionally affect the tool's performance.

All of the necessary cases have been tested.