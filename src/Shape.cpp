#include "Shape.h"


namespace utils {

double distanceBetweenPoints(const Point& lhs, const Point& rhs) {
  return sqrt((lhs.x - rhs.x) * (lhs.x - rhs.x) +
              (lhs.y - rhs.y) * (lhs.y - rhs.y));
}

bool Point::operator==(const Point& p) const {
  return (x == p.x && y == p.y);
}

bool Point::operator<(const Point &p) const {
  return (x < p.x ? true :
         (x > p.x ? false : y < p.y));
}

void Shape::addShape(const LineSegment &ls) {
  auto& element = std::get<std::vector<LineSegment>>(shapes_);
  element.emplace_back(std::move(ls));
}

void Shape::addShape(const Circle &c) {
  auto& element = std::get<std::vector<Circle>>(shapes_);
  element.emplace_back(std::move(c));
}

std::optional<Point> Shape::getIntersectionPoint(const LineSegment& lhs, const LineSegment& rhs) {
  /*
   * The algorithm below computes the intersection point of two line segments.
   * Basically there is a system of 2D linear equations, which have to be solved for t1 and t2,
   * when x and y are both equal (if that's possible).
   * Step-by-step:
   * 1) Test for parallel lines:
   *  - if the direction vectors of the line segments are scalar multiples,
   *    then the lines are parallel and can't intersect:
   *    s10_x = p1_x - p0_x
   *    s10_y = p1_y - p0_y
   *    s32_x = p3_x - p2_x
   *    s32_y = p3_y - p2_y
   *
   * 2) Compute the t and s values based on the following:
   *  - Two lines are given by the form p = p0 + v * t, specifically:
   *    x1 = p10_x + v1.x * t
   *    y1 = p10_y + v1.y * t
   *    x2 = p20_x + v2.x * s
   *    y2 = p20_y + v2.y * s
   *
   *  - Solve the above equations when x1 = x2 and y1 = y2
   *    As a result:
   *    s = (-s10_y * (p0_x - p2_x) + s10_x * (p0_y - p2_y)) / (-s32_x * s10_y + s10_x * s32_y)
   *    t = ( s32_x * (p0_y - p2_y) - s32_y * (p0_x - p2_x)) / (-s32_x * s10_y + s10_x * s32_y)
   *
   * 3) Check if the denominator is equal to zero
   *  - if it is zero, then the segments are collinear, so no intersection
   *
   * 4) There is only an intersection between the segments, if the following is true:
   *    0 <= t <= 1
   *    0 <= s <= 1
   *
   * 5) Calculate the intersection points based on the following equations:
   *  - x = p0_x + (t * s10_x)
   *  - y = p0_y + (t * s10_y)
   *
   * The function returns the intersection point if the segments intersect, otherwise nothing.
   */

  /* direction vectors (based on the segments) */
  double s10X = lhs.end.x - lhs.begin.x;
  double s10Y = lhs.end.y - lhs.begin.y;
  double s32X = rhs.end.x - rhs.begin.x;
  double s32Y = rhs.end.y - rhs.begin.y;
  double s02X = lhs.begin.x - rhs.begin.x;
  double s02Y = lhs.begin.y - rhs.begin.y;

  /* denominator of the parameter equations */
  double denom = -s32X * s10Y + s10X * s32Y;

  /* segments are collinear if the denominator equals to zero */
  if (std::fabs(0.0 - denom) < std::numeric_limits<double>::epsilon()) {
    return {};
  }

  /* solve the equations for s and t */
  /* numerator of equation s */
  double sNumerator = s10X * s02Y - s10Y * s02X;

  /* numerator of equation t */
  double tNumerator = s32X * s02Y - s32Y * s02X;

  /* calculate equation parameters */
  double s = sNumerator / denom;
  double t = tNumerator / denom;

  /* check validity for t and s */
  if (t >= 0.0 && t <= 1.0 && s >= 0.0 && s <= 1.0) {
    /* intersection detected */
    /* calculating intersection points */
    double x = lhs.begin.x + (t * s10X);
    double y = lhs.begin.y + (t * s10Y);

    return Point{x, y};
  }

  /* no intersection on the segments */
  return {};
}

std::vector<Point> Shape::getIntersectionPoint(const LineSegment& ls, const Circle& c) {
  /*
   * To find the points of intersection, the below algorithm considers the line as the following equations:
   * X(t) = x1 + (x2 - x1) * t
   * Y(t) = y1 + (y2 - y1) * t
   *
   * Additionally the algorithm plugs these two equations into the following equation for a circle:
   * (X - Cx)^2 + (Y - Cy)^2 = radius^2
   *
   * then it solves for t by using the quadratic formula.
   * The result is 0, 1, or 2 real values for t depending on whether the line cuts through the circle,
   * touches it tangentially, or misses it entirely.
   *
   * If the determinant is < 0, then there is no intersection point at all.
   * If 0 <= t <= 1, then there is an intersection points (1 or 2, based on the +-t values).
   */
  std::vector<Point> intersectionPoints;

  double deltaX = ls.end.x - ls.begin.x;
  double deltaY = ls.end.y - ls.begin.y;

  /* calculate coefficients for the quadratic formula */
  double A = (deltaX * deltaX) + (deltaY * deltaY);
  double B = 2 * (deltaX * (ls.begin.x - c.center.x) + deltaY * (ls.begin.y - c.center.y));
  double C = (ls.begin.x - c.center.x) * (ls.begin.x - c.center.x) +
             (ls.begin.y - c.center.y) * (ls.begin.y - c.center.y) -
             (c.radius * c.radius);

  /* calculate discriminant */
  double discriminant = (B * B) - (4 * A * C);

  /* if discriminant is less than 0, then there is no intersection */
  if (discriminant < 0.0) {
    return intersectionPoints;
  /* if discriminant is greater than 0, then there are one or two intersections */
  } else {
    /* first intersection point */
    double t = ((-B + sqrt(discriminant)) / (2 * A));

    /* the intersection point is valid only if 0 <= t <= 1 */
    if (t >= 0.0 && t <= 1.0) {
      double x = ls.begin.x + (t * deltaX);
      double y = ls.begin.y + (t * deltaY);
      intersectionPoints.emplace_back(Point{x, y});
    }

    /* second intersection point */
    t = ((-B - sqrt(discriminant)) / (2 * A));

    /* the intersection point is valid only if 0 <= t <= 1 */
    if (t >= 0.0 && t <= 1.0) {
      double x = ls.begin.x + (t * deltaX);
      double y = ls.begin.y + (t * deltaY);
      intersectionPoints.emplace_back(Point{x, y});
    }
  }

  return intersectionPoints;
}

std::vector<Point> Shape::getIntersectionPoint(const Circle& lhs, const Circle& rhs) {
  /*
   * The algorithm below finds the intersection points (if any) between two circles.
   *
   * Assume centers of the circle are (x1, y1) and (x2, y2), radiuses are R1 and R2.
   * Let the ends of the base be A and B and the target point be T. We know that AT = R1 and BT = R2.
   * The simplest trick to find T is to notice that difference of the squares of the distances is a known constant
   * (R1^2 - R2^2). And it is easy to see that the line that contains points meeting this condition is actually a straight line
   * perpendicular to the base. We also have the circles equations:
   *
   * (x - x1)^2 + (y-y1)^2 = R1^2
   * (x - x2)^2 + (y-y2)^2 = R2^2
   *
   * If we subtract one from another we'll get an equation, with the difference of the squares of the distances on the right side:
   * (x2 - x1)(2*x - x1 - x2) + (y2 - y1)(2*y - y1 - y2) = R1^2 - R2^2
   *
   * Let's x0 = (x1 + x2)/2 and y0 = (y1 + y2)/2 - the coordinates of the center. Let also the length of the base be L and
   * its projections dx = x2 - x1 and dy = y2 - y1 (i.e. L^2 = dx^2 + dy^2). And let's Q = R1^2 - R2^2. So we can see that:
   * 2 * (dx * (x-x0) + dy*(y-y0)) = Q
   *
   * So the line for all (x,y) pairs with R1^2 - R2^2 = Q = const is a straight line orthogonal to the base
   * (because coefficients are exactly dx and dy).
   * Let's find the point C on the base that is the intersection with that line. It splits the base so that difference of the squares
   * of the lengths is Q. It is easy to find out that it is the point on a distance L/2 + Q/(2*L) from A and L/2 - Q/(2*L) from B.
   * So now we can find that:
   * TC^2 = R1^2 - (L/2 + Q/(2*L))^2
   *
   * Substituting back Q and simplifying a bit we can find that, we get an equation for TC:
   * TC^2 = (2*L^2*R1^2 + 2*L^2*R2^2 + 2*R1^2*R2^2 - L^4 - R1^4 - R2^4) / (4*L^2)
   * So let's:
   * a = (R1^2 - R2^2)/(2*L)
   * b = sqrt(2*L^2*R1^2 + 2*L^2*R2^2 + 2*R1^2*R2^2 - L^4 - R1^4 - R2^4) / (2*L)
   *
   * Note that formula for b can also be written in a different form:
   * b = sqrt[(R1+R2+L)*(-R1+R2+L)*(R1-R2+L)*(R1+R2-L)] / (2*L)
   *
   * which looks quite similar to the Heron's formula. And this is not a surprise because b is effectively the length of the height
   * to the base AB from T in the triangle ABT so its length is 2*S/L where S is the area of the triangle.
   * And the triangle ABT obviously has sides of lengths L, R1 and R2 respectively.
   * To find the target T we need to move 'a' along the base and 'b' in a perpendicular direction.
   * So coordinates of T calculated from the middle of the segment are:
   * Xt = x0 + a * dx/L ± b * dy / L
   * Yt = y0 + a * dy/L ± b * dx / L
   * Here ± means that there are two solutions: one on either side of the base line.
   *
   * Step-by-step:
   * First it calculates the distance between the centers of the circles (D).
   * Conditions for intersection between two circles:
   * r0 + r1 > D and D > |r0 – r1|
   *
   * Based on this, it can decide, whether there are 2, 1 or 0 intersection points.
   * If there are at least 1 intersection, then:
   * It calculates the area of the triangle formed by the two circle centers and one of the intersection point.
   * The sides of this triangle are D, r0 and r1, the area is calculated by Heron's formula.
   * Also, using the equations for the circles given by:
   * (x - a)^2 + (y - b)^2 = r0^2
   * (x - c)^2 + (y - d)^2 = r1^2
   *
   * where:
   * a = lhs.center.x
   * b = lhs.center.y
   * c = rhs.center.x
   * d = rhs.center.y
   * r0 = lhs.radius
   * r1 = rhs.radius
   *
   * The intersection points between the circles can be calculated as follows:
   *
   *        a + c   (c - a)(r0^2 - r1^2)     b - d
   * x1,2 = ----- + -------------------- +- 2-----S
   *          2             2D^2              D^2
   *
   *        b + d   (d - b)(r0^2 - r1^2)     a - c
   * y1,2 = ----- + -------------------- -+ 2-----S
   *          2             2D^2              D^2
   *
   * where S is the Heron's formula and it is given by:
   *
   * S = (1/4) * sqrt((D + r0 + r1)(D + r0 - r1)(D - r0 + r1)(-D + r0 + r1))
   */

  std::vector<Point> intersectionPoints;

  /* calculate distance between the centers of the circles */
  double D = sqrt((lhs.center.x - rhs.center.x) * (lhs.center.x - rhs.center.x) +
                  (lhs.center.y - rhs.center.y) * (lhs.center.y - rhs.center.y));

  /* check conditions whether intersection points exist */
  if (((lhs.radius + rhs.radius) >= D) && (D >= std::fabs(lhs.radius - rhs.radius))) {
    /* intersection point(s) should exist */

    /* calculate area according to Heron's formula */
    double s1 = (D + lhs.radius + rhs.radius);
    double s2 = (D + lhs.radius - rhs.radius);
    double s3 = (D - lhs.radius + rhs.radius);
    double s4 = (-D + lhs.radius + rhs.radius);
    double S = sqrt(s1 * s2 * s3 * s4) / 4.0;

    /* calculate base parts for the x1,2 and y1,2 equations */
    double baseX = (lhs.center.x + rhs.center.x) / 2.0 +
                   (rhs.center.x - lhs.center.x) * (lhs.radius * lhs.radius - rhs.radius * rhs.radius) / (2.0 * D * D);
    double baseY = (lhs.center.y + rhs.center.y) / 2.0 +
                   (rhs.center.y - lhs.center.y) * (lhs.radius * lhs.radius - rhs.radius * rhs.radius) / (2.0 * D * D);

    /* calculate the +- parts of the x1,2 and y1,2 equations */
    double changingPartX = 2.0 * S * (lhs.center.y - rhs.center.y) / (D * D);
    double changingPartY = 2.0 * S * (lhs.center.x - rhs.center.x) / (D * D);

    /* if the +- parts are equal to 0, then there is only one intersection point */
    if (changingPartX == 0.0 && changingPartY == 0.0) {
      intersectionPoints.emplace_back(Point{baseX, baseY});
    } else {
      /* there are two intersection points */
      double x1 = baseX + changingPartX;
      double x2 = baseX - changingPartX;
      double y1 = baseY - changingPartY;
      double y2 = baseY + changingPartY;

      intersectionPoints.emplace_back(Point{x1, y1});
      intersectionPoints.emplace_back(Point{x2, y2});
    }
  } else {
    /* there is no intersection point at all */
    /* nothing to do here, it is just for safety reason */
  }

  return intersectionPoints;
}

std::vector<Point> Shape::getIntersectionPoints() {
  std::vector<Point> intersections;

  /* get the first and last elements of the shapes (i.e. LineSegment and Circle) */
  /* LineSegment */
  auto lsFirst = std::get<0>(shapes_).cbegin();
  auto lsLast = std::get<0>(shapes_).cend();
  /* Circle */
  auto cFirst = std::get<1>(shapes_).cbegin();
  auto cLast = std::get<1>(shapes_).cend();

  for (auto ls1 = lsFirst; ls1 != lsLast; ++ls1) {
    for (auto ls2 = std::next(ls1, 1); ls2 != lsLast; ++ls2) {
      /* get intersection points between LineSegments */
      auto intersectionPoint = getIntersectionPoint(*ls1, *ls2);

      /* check whether intersection has a valid point */
      if (intersectionPoint.has_value()) {
        intersections.emplace_back(*intersectionPoint);
      }
    }

    /* get intersection points between LineSegments and Circles */
    for (auto c = cFirst; c != cLast; ++c) {
      auto intersectionPoint = getIntersectionPoint(*ls1, *c);

      /* check whether intersection has points */
      if (!intersectionPoint.empty()) {
        for (auto& it : intersectionPoint) {
          intersections.emplace_back(it);
        }
      }
    }
  }

  /* get intersection points between circles */
  for (auto c1 = cFirst; c1 != cLast; ++c1) {
    for (auto c2 = std::next(c1, 1); c2 != cLast; ++c2) {
      auto intersectionPoint = getIntersectionPoint(*c1, *c2);

      /* check whether intersection has points */
      if (!intersectionPoint.empty()) {
        for (auto& it : intersectionPoint) {
          intersections.emplace_back(it);
        }
      }
    }
  }

  /* sort intersection points by ascending order */
  std::sort(intersections.begin(), intersections.end());

  /* delete the duplicates */
  intersections.erase(std::unique(intersections.begin(), intersections.end()), intersections.end());

  return intersections;
}

} // namespace utils
