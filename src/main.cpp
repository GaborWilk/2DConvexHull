#include "ConvexHull.h"
#include "Shape.h"

#include <iomanip>
#include <sstream>


using namespace utils;

int main() {
  /* initialize shape container */
  Shape shape;

  /* read number of shapes from stdin */
  uint16_t numOfShapes;
  std::cin >> numOfShapes;
  std::cin.sync();

  /* read shapes */
  for (uint16_t i = 0u; i < numOfShapes; ++i) {
    std::string line;
    std::getline(std::cin, line);
    std::istringstream iss(line);

    /* get shape type */
    char type;
    switch (iss >> type; type) {
      case 'C':
      {
        Circle c;
        iss >> c.center.x >> c.center.y >> c.radius;
        shape.addShape(c);
        break;
      }
      case 'L':
      {
        LineSegment ls;
        iss >> ls.begin.x >> ls.begin.y >> ls.end.x >> ls.end.y;
        shape.addShape(ls);
        break;
      }
      default:
        throw std::runtime_error(std::string("Invalid shape type: ") + type);
    }
  }

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

  return 0;
}
