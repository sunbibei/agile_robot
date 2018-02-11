
/*
 * test.cpp
 *
 *  Created on: Jan 9, 2018
 *      Author: bibei
 */
// #define TEST_TRAJ
#ifdef  TEST_TRAJ

#include "polynomial.h"
#include <adt/segmented.h>
#include <adt/discrete.h>

#include <iostream>

using namespace qr_control;

int main() {
  // Traj3dSp traj;

  PolyTraj3dSp poly1(new PolyTraj3d);
  ///! The Polynomial initialize style 1
  PolyTraj3d::CoeffMat mat;
  mat.resize(3, 4);
  mat << 1,     2,   3, 4,
      0.1, 0.2, 0.3, 0.4,
      0.01, 0.02, 0.03, 0.04;
  poly1->reset(mat);
  std::cout << "Poly1 test:" << std::endl;
  ///! sample(-4) :  -215 -21.5 -2.15
  std::cout << "sample(-4) : "  << poly1->sample(-4).transpose()  << std::endl;
  ///! sample(0.1):   1.234  0.1234 0.01234
  std::cout << "sample(0.1): "  << poly1->sample(0.1).transpose() << std::endl;
  ///! x =      +1.00 t^0     +2.00 t^1     +3.00 t^2     +4.00 t^3
  ///! y =      +0.10 t^0     +0.20 t^1     +0.30 t^2     +0.40 t^3
  ///! z =      +0.01 t^0     +0.02 t^1     +0.03 t^2     +0.04 t^3
  std::cout << "Polynomial: \n" << *poly1 << std::endl;

  PolyTraj3dSp poly2(new PolyTraj3d);
  ///! The Polynomial initialize style 2
  *poly2 << 1,     2,   3, 4,
            0.1, 0.2, 0.3, 0.4,
            0.01, 0.02, 0.03, 0.04;
  std::cout << "Poly2 test:" << std::endl;
  ///! sample(-1.5) : -8.75 -0.88 -0.09
  std::cout << "sample(-1.5) : "  << poly2->sample(-1.5).transpose() << std::endl;
  ///! sample(10)   : +4321.00  +432.10   +43.21
  std::cout << "sample(10)   : "  << poly2->sample(10).transpose()   << std::endl;
  ///! x =      +1.00 t^0     +2.00 t^1     +3.00 t^2     +4.00 t^3
  ///! y =      +0.10 t^0     +0.20 t^1     +0.30 t^2     +0.40 t^3
  ///! z =      +0.01 t^0     +0.02 t^1     +0.03 t^2     +0.04 t^3
  std::cout << "Poly2: \n"  << *poly2 << std::endl;

  Traj3dSp traj = poly2;
  std::cout << "Trajectory test:" << std::endl;
  ///! sample(-1.5) : -8.75 -0.88 -0.09
  std::cout << "sample(-1.5) : "  << poly2->sample(-1.5).transpose() << std::endl;
  ///! sample(10)   : +4321.00  +432.10   +43.21
  std::cout << "sample(10)   : "  << poly2->sample(10).transpose()   << std::endl;

  ///! The discrete
  DisTraj3dSp discrete(new DisTraj3d);
  for (double d = 0; d < 2; d += 0.1) {
    DisTraj3d::StateVec vec(d, d*d, d*d*d);
    discrete->push_back(d, vec);
  }
  DisTraj3d::StateVec vec(1, 1, 1);
  discrete->push_back(-0.5, vec);
  discrete->push_back(0.65, vec);
  discrete->push_back(0.25, vec);
  discrete->push_back(0.95, vec);
  discrete->push_back(1.45, vec);
  discrete->push_back(20, vec); // The range is [-0.5, 20]

  std::cout << "Discrete test:" << std::endl;
  ///! sample(-0.49) : +0.98 +0.98 +0.98
  std::cout << "sample(-0.49) : "  << discrete->sample(-0.49).transpose() << std::endl;
  ///! sample(-0.75) : +0.75 +0.56 +0.43
  std::cout << "sample(-0.75) : "  << discrete->sample(0.75).transpose()  << std::endl;
  ///! sample(-0.4)  : +0.80 +0.80 +0.80
  std::cout << "sample(-0.4)  : "  << discrete->sample(-0.4).transpose()  << std::endl;
  ///! sample(-10)   : +1.50 +2.44 +4.24
  std::cout << "sample(-10)   : "  << discrete->sample(10).transpose()    << std::endl;
  ///! sample(-65)   : +1.00 +1.00 +1.00
  std::cout << "sample(-65)   : "  << discrete->sample(-65).transpose()   << std::endl;
  ///! sample(65)    : +1.00 +1.00 +1.00
  std::cout << "sample(65)    : "  << discrete->sample(65).transpose()    << std::endl;
  // std::cout << *dis1 << std::endl;

  SegTraj3dSp seg(new SegTraj3d);
  ///! assert fail!
  // seg->add(poly1);
  seg->add(poly1, -5, -3);
  seg->add(poly2, -3, -0.5);
  seg->add(discrete);

  std::cout << "Segmented test:" << std::endl;
  ///! sample(-0.4)  : -215.00  -21.50   -2.15
  std::cout <<  "sample(-0.4)  : "  << seg->sample(-4).transpose() << std::endl;    // \in poly1
  ///! sample(-1.5)  : -8.75 -0.88 -0.09
  std::cout <<  "sample(-1.5)  : "  << seg->sample(-1.5).transpose() << std::endl;  // \in poly2
  ///! sample(-0.51) : +0.23 +0.02 +0.00
  std::cout <<  "sample(-0.51) : "  << seg->sample(-0.51).transpose() << std::endl; // \in poly2
  ///! sample(-0.49) : +0.98 +0.98 +0.98
  std::cout <<  "sample(-0.49) : "  << seg->sample(-0.49).transpose() << std::endl; // \in discrete
  ///! sample(-0.75) : +0.75 +0.56 +0.43
  std::cout <<  "sample(-0.75) : "  << seg->sample(0.75).transpose() << std::endl;  // \in discrete

  SegTraj3dSp seg1(new SegTraj3d);
  poly1->range(-5, -3);
  poly2->range(-3, -0.5);
  seg1->add(poly1);
  seg1->add(poly2);
  seg1->add(discrete);
  std::cout << "Segmented1 test:" << std::endl;
  ///! sample(-0.4)  : -215.00  -21.50   -2.15
  std::cout <<  "sample(-0.4)  : "  << seg1->sample(-4).transpose() << std::endl;    // \in poly1
  ///! sample(-1.5)  : -8.75 -0.88 -0.09
  std::cout <<  "sample(-1.5)  : "  << seg1->sample(-1.5).transpose() << std::endl;  // \in poly2
  ///! sample(-0.51) : +0.23 +0.02 +0.00
  std::cout <<  "sample(-0.51) : "  << seg1->sample(-0.51).transpose() << std::endl; // \in poly2
  ///! sample(-0.49) : +0.98 +0.98 +0.98
  std::cout <<  "sample(-0.49) : "  << seg1->sample(-0.49).transpose() << std::endl; // \in discrete
  ///! sample(-0.75) : +0.75 +0.56 +0.43
  std::cout <<  "sample(-0.75) : "  << seg1->sample(0.75).transpose() << std::endl;  // \in discrete

  traj = seg1;
  std::cout << "Trajectory test:" << std::endl;
  ///! sample(-0.4)  : -215.00  -21.50   -2.15
  std::cout <<  "sample(-0.4)  : "  << traj->sample(-4).transpose() << std::endl;    // \in poly1
  ///! sample(-1.5)  : -8.75 -0.88 -0.09
  std::cout <<  "sample(-1.5)  : "  << traj->sample(-1.5).transpose() << std::endl;  // \in poly2
  ///! sample(-0.51) : +0.23 +0.02 +0.00
  std::cout <<  "sample(-0.51) : "  << traj->sample(-0.51).transpose() << std::endl; // \in poly2
  ///! sample(-0.49) : +0.98 +0.98 +0.98
  std::cout <<  "sample(-0.49) : "  << traj->sample(-0.49).transpose() << std::endl; // \in discrete
  ///! sample(-0.75) : +0.75 +0.56 +0.43
  std::cout <<  "sample(-0.75) : "  << traj->sample(0.75).transpose() << std::endl;  // \in discrete


  return 0;
}

#endif
