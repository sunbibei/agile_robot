/*
 * trajectory.h
 *
 *  Created on: Jan 6, 2018
 *      Author: bibei
 */

#ifndef INCLUDE_ADT_TRAJECTORY_H_
#define INCLUDE_ADT_TRAJECTORY_H_

#include <foundation/utf.h>

#include <Eigen/Dense>
#include <boost/shared_ptr.hpp>

namespace agile_control {

/*!
 * @brief The trajectory base class, define the interface for each sub-class
 */
template<typename _DataType, int _Dim_X>
class Trajectory {
public:
  ///! Convenient alias and structure define.
  typedef boost::shared_ptr<Trajectory<_DataType, _Dim_X>> TrajSp;
  typedef Eigen::Matrix<_DataType, _Dim_X, 1>              StateVec;
  typedef Eigen::Matrix<_DataType, _Dim_X, Eigen::Dynamic> StateSeq;
  typedef struct {
    _DataType floor;
    _DataType ceiling;
  } Range;

public:
  Trajectory();
  Trajectory(const Trajectory&);
  virtual Trajectory& operator=(const Trajectory&);

  virtual ~Trajectory();

///! The sub-class must be implemented!
public:
  ///! This method will clear the trajectory.
  virtual void clear() /*= 0*/;
  ///! This method will set the coefficients.
  // virtual void reset(const TrajSp&)     = 0;
  ///! This method sample the trajectory at parameter _t
  virtual StateVec sample(_DataType _t) const = 0;

public:
  ///! This method products a sequence through discretizing the trajectory.
  virtual StateSeq sequence(_DataType _from, _DataType _to, _DataType _dt);
  ///! Differential at some point
  virtual StateVec differential(_DataType _t) /*= 0*/;
  ///! Differential trajectory object
  virtual TrajSp   differential()             /*= 0*/;
  ///! integral trajectory object under the given sample(_t0, _y0)
  virtual TrajSp   integral(const _DataType& _t0, const StateVec& _y0) /*= 0*/;

public:
  ///! This method will set the range of parameters @t.
  void range(_DataType _t0, _DataType _t1);
  _DataType clamp(_DataType _t) const;
  ///! This method return the start point
  _DataType floor(bool* exit = nullptr)   const;
  ///! This method return the end   point
  _DataType ceiling(bool* exit = nullptr) const;

protected:
  Range*  range_;
};

///////////////////////////////////////////////////////////////////////////////
//////////////           The helper method define first
///////////////////////////////////////////////////////////////////////////////
template<typename _DataType>
inline Eigen::Matrix<_DataType, Eigen::Dynamic, 1>
__get_state_vec(const _DataType& _t, const Eigen::Index& _exp);

template<typename _DataType>
inline _DataType __clamp(const _DataType& _t, const _DataType& _low, const _DataType& _hi);


///////////////////////////////////////////////////////////////////////////////
////////////        The implementation of template methods         ////////////
///////////////////////////////////////////////////////////////////////////////
template<typename _DataType, int _Dim_X>
Trajectory<_DataType, _Dim_X>::Trajectory()
  : range_(nullptr) {
  ;
}

template<typename _DataType, int _Dim_X>
Trajectory<_DataType, _Dim_X>::Trajectory(const Trajectory<_DataType, _Dim_X>& _o)
  : range_(nullptr) {
  if (_o.range_) {
    range_ = new Range;
    range_->ceiling = _o.range_->ceiling;
    range_->floor   = _o.range_->floor;
  }
}

template<typename _DataType, int _Dim_X>
Trajectory<_DataType, _Dim_X>&
Trajectory<_DataType, _Dim_X>::operator=(const Trajectory<_DataType, _Dim_X>& _o) {
  if (_o.range_) {
    if (!range_) range_ = new Range;
    range_->ceiling = _o.range_->ceiling;
    range_->floor   = _o.range_->floor;
  }
  return *this;
}

template<typename _DataType, int _Dim_X>
Trajectory<_DataType, _Dim_X>::~Trajectory() {
  delete range_;
  range_ = nullptr;
}

template<typename _DataType, int _Dim_X>
void
Trajectory<_DataType, _Dim_X>::clear() {
  delete range_;
  range_ = nullptr;
}

template<typename _DataType, int _Dim_X>
void Trajectory<_DataType, _Dim_X>::range(_DataType _t0, _DataType _t1) {
  if (!range_) range_ = new Range;

  range_->floor   = _t0;
  range_->ceiling = _t1;
}

template<typename _DataType, int _Dim_X>
_DataType Trajectory<_DataType, _Dim_X>::clamp(_DataType _t) const {
  if (!range_) return _t;

  return __clamp(_t, range_->floor, range_->ceiling);
}

template<typename _DataType, int _Dim_X>
_DataType Trajectory<_DataType, _Dim_X>::floor(bool* exit) const {
  if (exit) *exit = (nullptr != range_);

  return (range_) ? range_->floor : std::numeric_limits<_DataType>::min();
}

template<typename _DataType, int _Dim_X>
_DataType Trajectory<_DataType, _Dim_X>::ceiling(bool* exit) const {
  if (exit) *exit = (nullptr != range_);

  return (range_) ? range_->ceiling : std::numeric_limits<_DataType>::max();
}

template<typename _DataType, int _Dim_X>
typename Trajectory<_DataType, _Dim_X>::StateSeq
Trajectory<_DataType, _Dim_X>::sequence(
    _DataType _from, _DataType _to, _DataType _dt) {
  assert((_to > _from) && (0 != _dt));
  if (range_) {
    if (_from < range_->floor)   _from = range_->floor;
    if (_to   > range_->ceiling) _to   = range_->ceiling;
  }
  typename Trajectory<_DataType, _Dim_X>::StateSeq ret;
  ret.resize(_Dim_X, (_to - _from) / _dt + 1);

  Eigen::Index off = 0;
  for (_DataType _t = _from; _t < _to; _t += _dt)
    ret.col(off++) = sample(_t);

  return ret;
}

template<typename _DataType, int _Dim_X>
typename Trajectory<_DataType, _Dim_X>::StateVec
Trajectory<_DataType, _Dim_X>::differential(_DataType _t) {
  LOG_ERROR << "Call the 'Trajectory<_DataType, _Dim_X>::differential(t)' which has does not complemented.";
  return typename Trajectory<_DataType, _Dim_X>::StateVec();
}

template<typename _DataType, int _Dim_X>
typename Trajectory<_DataType, _Dim_X>::TrajSp
Trajectory<_DataType, _Dim_X>::differential() {
  LOG_ERROR << "Call the 'Trajectory<_DataType, _Dim_X>::differential()' which has does not complemented.";
  return nullptr;
}

template<typename _DataType, int _Dim_X>
typename Trajectory<_DataType, _Dim_X>::TrajSp
Trajectory<_DataType, _Dim_X>::integral(const _DataType& _t0, const StateVec& _y0) {
  LOG_ERROR << "Call the 'Trajectory<_DataType, _Dim_X>::integral()' which has does not complemented.";
  return nullptr;
}

///////////////////////////////////////////////////////////////////////////////
////////        The implementation of template helper methods         /////////
///////////////////////////////////////////////////////////////////////////////
template<typename _DataType>
inline Eigen::Matrix<_DataType, Eigen::Dynamic, 1> __get_state_vec(const _DataType& _t, const Eigen::Index& _exp) {
  Eigen::Matrix<_DataType, Eigen::Dynamic, 1> _vec;
  _vec.resize(_exp, 1);
  Eigen::Index idx = 0;
  _vec(idx++) = 1;
  for (; idx < _vec.rows(); ++idx) {
    _vec(idx) = _vec(idx-1)*_t;
  }
  return _vec;
}

template<typename _DataType>
inline _DataType __clamp(const _DataType& _t, const _DataType& _low, const _DataType& _hi) {
  return ( (_low < _hi) ? ( (_t < _low) ? _low : ( (_t > _hi ) ? (_hi ) : _t ) )
                        : ( (_t < _hi)  ? _hi  : ( (_t > _low) ? (_low) : _t ) ) );
}


///////////////////////////////////////////////////////////////////////////////
////////////                The convenient typedef                 ////////////
///////////////////////////////////////////////////////////////////////////////
#define TRAJ_MAKE_TYPEDEFS(Type, TypeSuffix, Size, SizeSuffix)   \
using Traj##SizeSuffix##TypeSuffix##Sp = Trajectory<Type, Size>::TrajSp;

#define TRAJ_MAKE_TYPEDEFS_ALL_SIZES(Type, TypeSuffix) \
TRAJ_MAKE_TYPEDEFS(Type, TypeSuffix, 1, 1) \
TRAJ_MAKE_TYPEDEFS(Type, TypeSuffix, 2, 2) \
TRAJ_MAKE_TYPEDEFS(Type, TypeSuffix, 3, 3) \
TRAJ_MAKE_TYPEDEFS(Type, TypeSuffix, 4, 4) \

//TRAJ_MAKE_TYPEDEFS_ALL_SIZES(int,    i)
//TRAJ_MAKE_TYPEDEFS_ALL_SIZES(char,   c)
//TRAJ_MAKE_TYPEDEFS_ALL_SIZES(short,  s)
//TRAJ_MAKE_TYPEDEFS_ALL_SIZES(float,  f)
TRAJ_MAKE_TYPEDEFS_ALL_SIZES(double, d)

#undef TRAJ_MAKE_TYPEDEFS_ALL_SIZES
#undef TRAJ_MAKE_TYPEDEFS

} /* namespace qr_control */

#endif /* INCLUDE_ADT_TRAJECTORY_H_ */
