/*
 * fourier.h
 *
 *  Created on: Apr 18, 2018
 *      Author: bibei
 */

#ifndef INCLUDE_ADT_FOURIER_H_
#define INCLUDE_ADT_FOURIER_H_

#include "trajectory.h"

#include <iomanip>

namespace agile_control {

/*!
 * @brief The Fourier series, formula as the follow:
 *  $$ f(X) = bias + \sum_{k=1}^{D}{a_{i,k}*sin(k\omega*X) + b_{i,k}*cos(k\omega*X)} $$
 *  $$ f(X) = \sum_{k=0}^{D}{ a_{i,k}*sin(k\omega*X) + b_{i,k}*cos(k\omega*X) }
 */
template<typename _DataType, int _Dim_X>
class Fourier: public Trajectory<_DataType, _Dim_X> {
public:
  ///! Convenient alias
  typedef Eigen::Matrix<_DataType, _Dim_X, Eigen::Dynamic> CoeffMat;

public:
  /*!
   * @brief The default constructor which nothing to do.
   */
  Fourier();
  /*!
   * @brief The constructor.
   * @param coeff The list of coefficient.
   */
  Fourier(const CoeffMat& _s, const CoeffMat& _c, const _DataType& _o);
  /*!
   * @brief The constructor.
   * @param coeff The list of coefficient.
   */
  Fourier(const std::vector<_DataType>& _s, const std::vector<_DataType>& _c, const _DataType& _o);
  /*!
   * @brief The copy constructor.
   * @param coeff The list of coefficient.
   */
  Fourier(const Fourier<_DataType, _Dim_X>&);
  /*!
   * @brief The assignment operators.
   * @param coeff The list of coefficient.
   */
  Fourier<_DataType, _Dim_X>& operator=(const Fourier<_DataType, _Dim_X>&);
  /*!
   * @brief The deconstructor
   */
  virtual ~Fourier() { }

private:
  ///! the coefficient of sin or cos, namely a_{i,k} or b_{i, k}
  ///! _Dim_X by dim_exp，each rows are a element of StateVec.
  CoeffMat     sin_coeffs_;
  CoeffMat     cos_coeffs_;
  ///! the base frequency
  _DataType    omega_;
  ///! the size of expand
  Eigen::Index dim_exp_;

public:
  ///! This method will set the coefficients.
  void reset(const CoeffMat&, const CoeffMat&, const _DataType& omega);
  void reset(
      const std::vector<_DataType>&,
      const std::vector<_DataType>&,
      const _DataType& omega);

public:
  ///! This method will clear the trajectory.
  virtual void clear() override;
  ///! This method sample the trajectory at parameter _t
  virtual typename Trajectory<_DataType, _Dim_X>::StateVec sample(_DataType _t) const override;
  ///! Differential at some point
  virtual typename Trajectory<_DataType, _Dim_X>::StateVec differential(_DataType _t) override;
  ///! Differential trajectory object
  virtual typename Trajectory<_DataType, _Dim_X>::TrajSp   differential() override;

public:
  template<typename _T1>
  friend std::ostream& operator<<(std::ostream& os, const Fourier<_T1, 1>& traj);
  template<typename _T1>
  friend std::ostream& operator<<(std::ostream& os, const Fourier<_T1, 2>& traj);
  template<typename _T1>
  friend std::ostream& operator<<(std::ostream& os, const Fourier<_T1, 3>& traj);
  template<typename _T1, int _T2>
  friend std::ostream& operator<<(std::ostream&, const Fourier<_T1, _T2>& traj);
};


///////////////////////////////////////////////////////////////////////////////
////////        The implementation of template helper methods         /////////
///////////////////////////////////////////////////////////////////////////////
template<typename _DataType>
inline Eigen::Matrix<_DataType, Eigen::Dynamic, 1> __get_sin_vec(
    const _DataType& _t, const Eigen::Index& _exp, const _DataType& omega) {
  Eigen::Matrix<_DataType, Eigen::Dynamic, 1> _vec;
  _vec.resize(_exp, 1);

  for (Eigen::Index idx = 0; idx < _exp; ++idx) {
    _vec(idx) = sin((double)idx * omega * _t);
  }

  // std::cout << "sin: " << _vec.transpose() << std::endl;
  return _vec;
}

template<typename _DataType>
inline Eigen::Matrix<_DataType, Eigen::Dynamic, 1> __get_diff_sin_vec(
    const _DataType& _t, const Eigen::Index& _exp, const _DataType& omega) {
  Eigen::Matrix<_DataType, Eigen::Dynamic, 1> _vec;
  _vec.resize(_exp, 1);

  for (Eigen::Index idx = 0; idx < _exp; ++idx) {
    _vec(idx) = (double)idx * omega * cos((double)idx * omega * _t);
  }
  return _vec;
}

template<typename _DataType>
inline Eigen::Matrix<_DataType, Eigen::Dynamic, 1> __get_cos_vec(
    const _DataType& _t, const Eigen::Index& _exp, const _DataType& omega) {
  Eigen::Matrix<_DataType, Eigen::Dynamic, 1> _vec;
  _vec.resize(_exp, 1);

  for (Eigen::Index idx = 0; idx < _exp; ++idx) {
    _vec(idx) = cos((double)idx * omega * _t);
  }
  // std::cout << "cos: "  << _vec.transpose() << std::endl;
  return _vec;
}

template<typename _DataType>
inline Eigen::Matrix<_DataType, Eigen::Dynamic, 1> __get_diff_cos_vec(
    const _DataType& _t, const Eigen::Index& _exp, const _DataType& omega) {
  Eigen::Matrix<_DataType, Eigen::Dynamic, 1> _vec;
  _vec.resize(_exp, 1);

  for (Eigen::Index idx = 0; idx < _exp; ++idx) {
    _vec(idx) = (double)idx * omega * (-sin((double)idx * omega * _t));
  }
  return _vec;
}

template<typename _DataType, int _Dim_X>
Fourier<_DataType, _Dim_X>::Fourier()
  : Trajectory<_DataType, _Dim_X>(), dim_exp_(0) {
  ; // Nothing to do here.
}

template<typename _DataType, int _Dim_X>
Fourier<_DataType, _Dim_X>::Fourier(const CoeffMat& _s, const CoeffMat& _c, const _DataType& _o)
  : Trajectory<_DataType, _Dim_X>(),
    cos_coeffs_(_c), sin_coeffs_(_s), omega_(_o) {
  assert( (_s.rows() == _Dim_X) && (_c.rows() == _Dim_X) );
  dim_exp_ = _s.cols();
}

template<typename _DataType, int _Dim_X>
Fourier<_DataType, _Dim_X>::Fourier(
    const std::vector<_DataType>& _s, const std::vector<_DataType>& _c, const _DataType& _o)
  : Trajectory<_DataType, _Dim_X>(), dim_exp_(0) {
  reset(_s, _c, _o);
}


template<typename _DataType, int _Dim_X>
Fourier<_DataType, _Dim_X>::Fourier(const Fourier<_DataType, _Dim_X>& _o)
  : Trajectory<_DataType, _Dim_X>(_o), sin_coeffs_(_o.sin_coeffs_),
    cos_coeffs_(_o.cos_coeffs_), omega_(_o.omega_),
    dim_exp_(_o.dim_exp_) {
  ;
}

template<typename _DataType, int _Dim_X>
Fourier<_DataType, _Dim_X>&
Fourier<_DataType, _Dim_X>::operator=(const Fourier<_DataType, _Dim_X>& _o) {
  this->operator =(_o);
  dim_exp_ = _o.dim_exp_;
  sin_coeffs_  = _o.sin_coeffs_;
  cos_coeffs_  = _o.cos_coeffs_;
  omega_       = _o.omega_;
  return *this;
}

template<typename _DataType, int _Dim_X>
void Fourier<_DataType, _Dim_X>::reset(
    const CoeffMat& _new_sin, const CoeffMat& _new_cos, const _DataType& omega) {
  assert( (_new_sin.rows() == _Dim_X) && (_new_cos.rows() == _Dim_X) );
  dim_exp_   = _new_sin.cols();

  sin_coeffs_    = _new_sin;
  cos_coeffs_    = _new_cos;
  omega_         = omega;
}

template<typename _DataType, int _Dim_X>
void Fourier<_DataType, _Dim_X>::reset(
    const std::vector<_DataType>& _new_sin,
    const std::vector<_DataType>& _new_cos,
    const _DataType& omega) {
  assert( (0 == (_new_sin.size()%_Dim_X))
      && (0 == (_new_cos.size()%_Dim_X))
      && ((_new_sin.size()%_Dim_X) == (_new_cos.size()%_Dim_X)) );

  omega_   = omega;
  dim_exp_ = _new_sin.size() / _Dim_X;
  sin_coeffs_.resize(_Dim_X, dim_exp_);

  for (int r = 0; r < _Dim_X; ++r)
    for (int c = 0; c < dim_exp_; ++c) {
      sin_coeffs_(r, c) = _new_sin[r*dim_exp_ + c];
      cos_coeffs_(r, c) = _new_cos[r*dim_exp_ + c];
    }
}

template<typename _DataType, int _Dim_X>
void Fourier<_DataType, _Dim_X>::clear() {
  sin_coeffs_.fill(0.0);
  cos_coeffs_.fill(0.0);
  omega_   = 0.0;
  dim_exp_ = 0;
  Trajectory<_DataType, _Dim_X>::clear();
}

template<typename _DataType, int _Dim_X>
typename Trajectory<_DataType, _Dim_X>::StateVec
Fourier<_DataType, _Dim_X>::sample(_DataType _t) const {
  assert( (0 != sin_coeffs_.cols()) && (0 != cos_coeffs_.cols()) );
  _t = Trajectory<_DataType, _Dim_X>::clamp(_t);

  return ( sin_coeffs_ * __get_sin_vec<_DataType>(_t, dim_exp_, omega_)
      + cos_coeffs_ * __get_cos_vec<_DataType>(_t, dim_exp_, omega_) );
}

template<typename _DataType, int _Dim_X>
typename Trajectory<_DataType, _Dim_X>::StateVec
Fourier<_DataType, _Dim_X>::differential(_DataType _t) {
  return ( sin_coeffs_ * __get_diff_sin_vec<_DataType>(_t, dim_exp_, omega_)
      + cos_coeffs_ * __get_diff_cos_vec<_DataType>(_t, dim_exp_, omega_) );
}

template<typename _DataType, int _Dim_X>
typename Trajectory<_DataType, _Dim_X>::TrajSp
Fourier<_DataType, _Dim_X>::differential() {
  auto _copy = new Fourier<_DataType, _Dim_X>(*this);

  auto diff_vec = __get_diff_cos_vec<_DataType>(0, _copy->dim_exp_, _copy->omega_);
  _copy->cos_coeffs_ = _copy->cos_coeffs_.cwiseProduct(diff_vec);

  for (Eigen::Index idx = 0; idx < _copy->dim_exp_; ++idx) {
    _copy->sin_coeffs_(idx) *= ( (double)idx * _copy->omega_ );
  }

  _copy->cos_coeffs_.swap(_copy->sin_coeffs_);
  return typename Trajectory<_DataType, _Dim_X>::TrajSp(_copy);
}

template<typename _T1, int _T2>
std::ostream& operator<<(std::ostream& os, const Fourier<_T1, _T2>& traj) {
  os.setf(std::ios::showpos | std::ios::fixed/* | std::ios::internal*/);
  if (traj.range_)
    os << "t \\in [" << traj.range_->floor << ", " << traj.range_->ceiling << "]\n";

  for (int i = 0; i < _T2; ++i) {
    os << "x" << std::to_string(i) << " = ";
    const auto& _coeff = traj.coeffs_.row(i);
    for (int j = 0; j < _coeff.cols(); ++j) {
      os << std::setw(10) << std::setprecision(2) << _coeff(j);
      os << " t^" << std::to_string(j);
    }
//    if (traj.range_) os << ",\tt \\in (" << traj.range_->floor
//        << ", " << traj.range_->ceiling << ")";
    os << "\n";
  }
  return os;
}

template<typename _T1>
std::ostream& operator<<(std::ostream& os, const Fourier<_T1, 1>& traj) {
  os.setf(std::ios::showpos | std::ios::fixed/* | std::ios::internal*/);
  if (traj.range_)
    os << "t \\in [" << traj.range_->floor << ", " << traj.range_->ceiling << "]\n";

  os << "x = ";
  const auto& _s = traj.sin_coeffs_.row(0);
  for (int j = 0; j < _s.cols(); ++j) {
    os << std::setw(10) << std::setprecision(2) << _s(j);
    os << " sin(" << std::to_string(j) << "*"
       << std::to_string(traj.omega_) << "*t)";
  }
  const auto& _c = traj.cos_coeffs_.row(0);
  for (int j = 0; j < _c.cols(); ++j) {
    os << std::setw(10) << std::setprecision(2) << _c(j);
    os << " cos(" << std::to_string(j) << "*"
       << std::to_string(traj.omega_) << "*t)";
  }
//  if (traj.range_) os << ",\tt \\in (" << traj.range_->floor
//      << ", " << traj.range_->ceiling << ")";
  os << "\n";

  return os;
}

template<typename _T1>
std::ostream& operator<<(std::ostream& os, const Fourier<_T1, 2>& traj) {
  os.setf(std::ios::showpos | std::ios::fixed/* | std::ios::internal*/);
  if (traj.range_)
    os << "t \\in [" << traj.range_->floor << ", " << traj.range_->ceiling << "]\n";

  for (int i = 0; i < 2; ++i) {
    os << (const char* []) {"x = ", "y = "}[i];
    const auto& _s = traj.sin_coeffs_.row(i);
    for (int j = 0; j < _s.cols(); ++j) {
      os << std::setw(10) << std::setprecision(2) << _s(j);
      os << " sin(" << std::to_string(j) << "*"
         << std::to_string(traj.omega_) << "*t)";
    }
    const auto& _c = traj.cos_coeffs_.row(i);
    for (int j = 0; j < _c.cols(); ++j) {
      os << std::setw(10) << std::setprecision(2) << _c(j);
      os << " cos(" << std::to_string(j) << "*"
         << std::to_string(traj.omega_) << "*t)";
    }
//    if (traj.range_) os << ",\tt \\in (" << traj.range_->floor
//        << ", " << traj.range_->ceiling << ")";
    os << "\n";
  }
  // os << "The coefficients of the trajectory is :\n" << traj.coeffs_ << std::endl;
  return os;
}

template<typename _T1>
std::ostream& operator<<(std::ostream& os, const Fourier<_T1, 3>& traj) {
  os.setf(std::ios::showpos | std::ios::fixed/* | std::ios::internal*/);
  if (traj.range_)
    os << "t \\in [" << traj.range_->floor << ", " << traj.range_->ceiling << "]\n";

  for (int i = 0; i < 3; ++i) {
    os << (const char* []) {"x = ", "y = ", "z = "}[i];
    const auto& _s = traj.sin_coeffs_.row(i);
    for (int j = 0; j < _s.cols(); ++j) {
      os << std::setw(10) << std::setprecision(2) << _s(j);
      os << " sin(" << std::to_string(j) << "*"
         << std::to_string(traj.omega_) << "*t)";
    }
    const auto& _c = traj.cos_coeffs_.row(i);
    for (int j = 0; j < _c.cols(); ++j) {
      os << std::setw(10) << std::setprecision(2) << _c(j);
      os << " cos(" << std::to_string(j) << "*"
         << std::to_string(traj.omega_) << "*t)";
    }
//    if (traj.range_) os << ",\tt \\in (" << traj.range_->floor
//        << ", " << traj.range_->ceiling << ")";
    os << "\n";
  }
  // os << "The coefficients of the trajectory is :\n" << traj.coeffs_ << std::endl;
  return os;
}



#define FOURIER_MAKE_TYPEDEFS(Type, TypeSuffix, Size, SizeSuffix)   \
using Fourier##SizeSuffix##TypeSuffix##Sp = boost::shared_ptr<Fourier<Type, Size>>; \
using Fourier##SizeSuffix##TypeSuffix     = Fourier<Type, Size>;

#define FOURIER_MAKE_TYPEDEFS_ALL_SIZES(Type, TypeSuffix) \
    FOURIER_MAKE_TYPEDEFS(Type, TypeSuffix, 1, 1) \
    FOURIER_MAKE_TYPEDEFS(Type, TypeSuffix, 2, 2) \
    FOURIER_MAKE_TYPEDEFS(Type, TypeSuffix, 3, 3) \
    FOURIER_MAKE_TYPEDEFS(Type, TypeSuffix, 4, 4)

//POLY_MAKE_TYPEDEFS_ALL_SIZES(int,    i)
//POLY_MAKE_TYPEDEFS_ALL_SIZES(char,   c)
//POLY_MAKE_TYPEDEFS_ALL_SIZES(short,  s)
//POLY_MAKE_TYPEDEFS_ALL_SIZES(float,  f)
FOURIER_MAKE_TYPEDEFS_ALL_SIZES(double, d)

#undef FOURIER_MAKE_TYPEDEFS_ALL_SIZES
#undef FOURIER_MAKE_TYPEDEFS

} /* namespace agile_control */

#endif /* INCLUDE_ADT_FOURIER_H_ */
