/*
 * geometry.cpp
 *
 *  Created on: Jan 12, 2018
 *      Author: bibei
 */

#include "adt/geometry.h"

namespace qr_control {
namespace geometry {

///! the distance between the point p1 to the point p2
double  distance(const Point2d& p1, const Point2d& p2) {
  return (p1 - p2).norm();
}

///! the linear cross both the a and b, if a == b, return (0, 0, 0);
Linear  linear(const Point2d& p0, const Point2d& p1) {
  if (p0 == p1)         return Linear(0, 0, 0);
  if (p0.x() == p1.x()) return Linear(1, 0, -p0.x());

  double k = (p1.y() - p0.y()) / (p1.x() - p0.x());
  double b = p0.y() - k*p0.x();
  return Linear(k, -1, b);
}

///! the linear cross the point a with the slope k;
Linear  linear(double k, const Point2d& p0) {
  return Linear(k, -1, p0.y() - k*p0.x());
}

///! the cross point of l1 and l2, if these is not exist cross point, return (NaN, NaN);
Point2d cross_point(const Linear& l1, const Linear& l2) {
  Eigen::Matrix2d _cof;
  _cof << l1.x(), l1.y(),
          l2.x(), l2.y();
  Eigen::Vector2d _b(-l1.z(), -l2.z());

  return (0 == _cof.determinant()) ? (Point2d(NAN, NAN))
        : (_cof.partialPivLu().solve(_b));
}

///! the distance between the point p to the linear l.
double  distance(const Linear& l, const Point2d& p) {
  return std::abs( (l.head(2).dot(p) + l.z()) ) / l.head(2).norm();
}

///! the angle between Linear l1 and l2.
double  angle(const Linear& l1, const Linear& l2) {
  return std::acos(std::abs(l1.head(2).dot(l2.head(2))) / (l1.head(2).norm()*l2.head(2).norm()));
}

double __cross_val(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2) {
  return p1.x()*p2.y() - p1.y()*p2.x();
}
///! whether in the triangle.
bool    is_in_triangle          (const Point2d& A, const Point2d& B, const Point2d& C, const Point2d& p) {
  double ca = __cross_val(C-A, p-A);
  double ba = __cross_val(B-A, p-A);
  double cb = __cross_val(C-B, p-B);
  if (ca * ba > 0) return false; // same +, or same -
  if ((0 == ca) && (0 == ba)) return true;
  if (0 == cb) return true;
  if (0 == ba) {
    if (ca * cb > 0) return false;
    else return true;
  }
  if (ba * cb > 0) return true;
  else return false;
}

///! the area of triangle.
double  area_of_triangle        (const Point2d& a, const Point2d& b, const Point2d& c) {
  Eigen::Matrix3d A = Eigen::Matrix3d::Ones();
  A.row(0).head(2) = a;
  A.row(1).head(2) = b;
  A.row(2).head(2) = c;
  return 0.5*std::abs(A.determinant());
}

///! the incenter of triangle.（内心）
Point2d incenter_of_triangle    (const Point2d& A, const Point2d& B, const Point2d& C) {
  ///! lens = (a, b, c)
  Eigen::Vector3d lens(distance(B, C), distance(A, C), distance(A, B));
  Eigen::Vector3d xs(A.x(), B.x(), C.x());
  Eigen::Vector3d ys(A.y(), B.y(), C.y());
  return Point2d( (lens.dot(xs)), (lens.dot(ys)) ) / lens.sum();
}

///! the orthocentre of triangle（垂心）
Point2d orthocentre_of_triangle (const Point2d& A, const Point2d& B, const Point2d& C) {
  Eigen::Vector2d a = C - B;
  Eigen::Vector2d b = A - C;
  Eigen::Vector2d c = B - A;
  double alpha = (a.dot(b))*(a.dot(c));
  double beta  = (b.dot(c))*(b.dot(a));
  double lamda = (c.dot(a))*(c.dot(b));
  return (alpha*A + beta*B + lamda*C)/(alpha+beta+lamda);
}

///! the circumcentre of triangle（外心）
Point2d circumcentre_of_triangle(const Point2d& A, const Point2d& B, const Point2d& C) {
//  Eigen::Vector2d a = C - B;
//  Eigen::Vector2d b = A - C;
//  Eigen::Vector2d c = B - A;
//  double d = 2 * std::pow(c.cross(a), 2);
//  double alpha = (a*a)*(c*b)/d;
//  double beta  = (b*b)*(c*a)/d;
//  double lamda = (c*c)*(b*a)/d;
//  return alpha*A + beta*B + lamda*C;
  return Point2d(0, 0);
}

///! the centroid of triangle（重心）
Point2d centroid_of_triangle    (const Point2d& a, const Point2d& b, const Point2d& c) {
  Eigen::Vector3d xs(a.x(), b.x(), c.x());
  Eigen::Vector3d ys(a.y(), b.y(), c.y());
  return Point2d( (xs.sum()/3.0), (ys.sum()/3.0) );
}

} /* end namespace geometry */
} /* end namespace qr_control */

// #define TEST_GEOMETRY
#ifdef TEST_GEOMETRY

using namespace qr_control;
using namespace geometry;
#include <iostream>

int main() {
  Point2d A(1, 1);
  Point2d B(1, 2);
  std::cout << "distance: " << distance(A, B) << std::endl;
  std::cout << "linear:   " << linear(A, B).transpose() << std::endl;

  Point2d C(2, 2);
  std::cout << "linear:   " << linear(A, C).transpose() << std::endl;
  std::cout << "cs point: " << cross_point(linear(A, B), linear(A, C)).transpose() << std::endl;
  std::cout << "pl dis:   " << distance(linear(A, B), C) << std::endl;
  std::cout << "pl dis:   " << distance(linear(A, B), A) << std::endl;
  std::cout << "angle:    " << angle(linear(A, B), linear(A, C)) << std::endl;
  std::cout << "SoT:      " << area_of_triangle(A, B, C) << std::endl;
  std::cout << "SoT:      " << area_of_triangle(C, B, A) << std::endl;
  std::cout << "IoT:      " << incenter_of_triangle(C, B, A).transpose() << std::endl;
  std::cout << "CcoT:     " << circumcentre_of_triangle(C, B, A).transpose() << std::endl;
  std::cout << "CtoT:     " << centroid_of_triangle(C, B, A).transpose() << std::endl;
  std::cout << "OoT:      " << orthocentre_of_triangle(C, B, A).transpose() << std::endl;
}

#endif
