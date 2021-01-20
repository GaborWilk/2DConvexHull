#include "ConvexHull.h"


namespace utils {

inline double ConvexHull::orientation(const Point& p1, const Point& p2, const Point& p3) {
  /*
   * The algorithm gets the slopes of the the point-pairs:
   * slope of (p1, p3): σ = (y3 − y1) / (x3 − x1)
   * slope of (p1, p2): τ = (y2 − y1) / (x2 − x1)
   *
   * Then tests the orientation with the following expression:
   * (y3 - y1)(x2 - x1) - (y2 - y1)(x3 - x1)
   *
   * returns > 0, if counter-clockwise (left turn)
   * returns < 0, if clockwise (right turn)
   * returns 0, if collinear
   */

  return (p3.y - p1.y) * (p2.x - p1.x) - (p2.y - p1.y) * (p3.x - p1.x);
}

void ConvexHull::storeHullPoints(const std::vector<Point>& hullPoints) {
  hullPoints_ = hullPoints;
}

std::vector<Point> ConvexHull::getConvexHull() {
  /*
   * The algorithm below is using the Graham Scan approach to calculate the convex hull points.
   *
   * It finds the lowest y-coordinate and leftmost point, called reference point.
   * It sorts the intersection points by increasing order of polar angle based on the reference point.
   * The algorithm proceeds by considering each of the points in the sorted array in sequence.
   * For each point, it is first determined whether traveling from the two points immediately preceding this point constitutes making
   * a left turn or a right turn. If a right turn, the second-to-last point is not part of the convex hull, and lies 'inside' it.
   * The same determination is then made for the set of the latest point and the two points that immediately precede the point found
   * to have been inside the hull, and is repeated until a "left turn" set is encountered,
   * at which point the algorithm moves on to the next point in the set of points in the sorted array minus any points that were found
   * to be inside the hull, because there is no need to consider these points again.
   *
   *
   * The implementation uses std::vector instead of std::stack for convenience.
   * Step-by-step:
   * 1) it finds the lowest y-coordinate and leftmost point, called reference point.
   * 2) it sorts the intersection points by polar angle based on the reference point.
   * 3) for every intersection points (except the reference point) it does the following:
   *    a) pop the last element from the 'stack' if we turn clockwise to reach this point
   *    b) while the 'stack' has more than 1 elements and the orientiation of
   *       next-to-top element, top element and actual intersection point is not counter-clockwise turn:
   *       remove top element from 'stack'
   *    c) add actual intersection point to 'stack'
   *
   * At the end, the algorithm returns an std::vector with all of the convex hull points,
   * or an empty container if one of the preconditions is not fulfilled.
   */

  std::vector<Point> hull;

  /* precondition check whether there are at least 3 intersection points */
  if (intersectionPoints_.size() < 3u) {
    return {};
  }

  /* if all intersection points are collinear, then the hull size shall be 0 */
  bool areCollinear = true;
  while (areCollinear) {
    for (size_t i = 0u; i < intersectionPoints_.size() - 2u; ++i) {
      if (orientation(intersectionPoints_[i], intersectionPoints_[i + 1], intersectionPoints_[i + 2]) != 0.0) {
        areCollinear = false;
        break;
      }
    }

    if (areCollinear) {
      return {};
    }
  }

  /* find lowest y-coordinate and leftmost point */
  auto minPosOfY = std::min_element(intersectionPoints_.begin(), intersectionPoints_.end(),
                               [](const Point& lhs, const Point& rhs) {
                                 return (lhs.y < rhs.y ? true :
                                        (lhs.y > rhs.y ? false : lhs.x < rhs.x));
                               });

  /* put lowest y-coordinate point to the front */
  std::iter_swap(minPosOfY, intersectionPoints_.begin());

  /* get the reference point */
  auto refPoint = intersectionPoints_[0];

  /* sort points by polar angle based on the reference point */
  std::sort(intersectionPoints_.begin() + 1, intersectionPoints_.end(),
            [&](const Point& lhs, const Point& rhs) {
              return std::atan2(lhs.y - refPoint.y, lhs.x - refPoint.x) <
                     std::atan2(rhs.y - refPoint.y, rhs.x - refPoint.x);
            });

  /* insert the first three points to the 'stack' */
  hull.emplace_back(intersectionPoints_[0]);
  hull.emplace_back(intersectionPoints_[1]);
  hull.emplace_back(intersectionPoints_[2]);

  /* get convex hull points */
  for (size_t i = 3u; i < intersectionPoints_.size(); ++i) {
    /* while there are more than 1 element in the 'stack' and the orientation is not counter-clockwise */
    while (hull.size() > 1u &&
           orientation(*(hull.rbegin() + 1), hull.back(), intersectionPoints_[i]) <= 0.0) {
      /* remove next element from the 'stack' */
      hull.pop_back();
    }

    /* insert actual point back to the 'stack' */
    hull.emplace_back(intersectionPoints_[i]);
  }

  /* there must be at least 3 hull points */
  if (hull.size() < 3) {
    hull.clear();
  }

  /* store hull points internally for future usage */
  storeHullPoints(hull);

  return hull;
}

double ConvexHull::getArea() {
  /* The algorithm below calculates twice the convex hull area.
   *
   * It uses Shoelace forumla or so called Gauss's area algorithm.
   * The area formula is derived by taking each edge AB from the points, and calculating the area of triangle ABO with a vertex
   * at the origin O, by taking the cross-product (which gives the area of a parallelogram) and dividing by 2.
   * As one wraps around the polygon, these triangles with positive and negative area will overlap,
   * and the areas between the origin and the polygon will be cancelled out and sum to 0,
   * while only the area inside the reference triangle remains.
   *
   * Step-by-step:
   * 1) Let 'vertices' be an array of N pairs (x,y), indexed from 0
   * 2) Let 'area' = 0.0
   * 3) for i = 0 to N-1, do
   *      a) Let j = (i+1) mod N
   *      b) Let area = area + vertices[i].x * vertices[j].y
   *      c) Let area = area - vertices[i].y * vertices[j].x
   * 4) Divide area by 2
   *
   * If the vertices of the polygon are specified in counter-clockwise order,
   * then the area will be positive. Otherwise, the area will be negative.
   * This approach is simpler, more compact and faster than using triangulation.
   */

  double areaTwice = 0.0;

  if (!hullPoints_.empty()) {
    for (size_t i = 0u; i < hullPoints_.size(); ++i) {
      size_t j = (i + 1u) % hullPoints_.size();
      areaTwice += hullPoints_[i].x * hullPoints_[j].y;
      areaTwice -= hullPoints_[i].y * hullPoints_[j].x;
    }
  }

  return areaTwice / 2.0;
}

} // namespace utils
