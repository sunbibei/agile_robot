/*
 * dis_trajectory.h
 *
 *  Created on: Jan 9, 2018
 *      Author: bibei
 */

#ifndef INCLUDE_ADT_DISCRETE_H_
#define INCLUDE_ADT_DISCRETE_H_

#include "trajectory.h"
#include <fstream>

namespace agile_control {

/*!
 * @brief The implement of discrete trajectory.
 */
template<typename _DataType, int _Dim_X>
class Discrete : public Trajectory<_DataType, _Dim_X> {
public:
  /*!
   * @brief The default constructor.
   */
  Discrete(int reserve_size = 128);

  virtual ~Discrete();

public:
  ///! This method will clear the trajectory.
  virtual void clear() override;
  ///! This method sample the trajectory at parameter _t
  virtual typename Trajectory<_DataType, _Dim_X>::StateVec sample(_DataType _t) const override;
  ///! Differential at some point
  // virtual StateVec differential(_DataType _t) override;
  ///! Differential trajectory object
  // virtual Trajectory<_DataType, _Dim_X> differential() override;
  ///! integral trajectory object under given sample
  // virtual Trajectory<_DataType, _Dim_X> integral(const _DataType& _t0, const StateVec& _y0) override;

  ///! This method will set the coefficients.
  void push_back(_DataType, const typename Trajectory<_DataType, _Dim_X>::StateVec&);
  ///! This method remove a segmented trajectory
  // void remove(Trajectory<_DataType, _Dim_X>*);
  ///! A especial method for save the trajectory to local file.
  void save(const std::string&);

public:
  template<typename _T1, int _T2>
  friend std::ostream& operator<<(std::ostream&, const Discrete<_T1, _T2>& traj);

protected:
  MiiMap<_DataType, typename Trajectory<_DataType, _Dim_X>::StateVec> seqs_;
  MiiVector<_DataType> sorted_t_set_;
};


///////////////////////////////////////////////////////////////////////////////
////////////        The implementation of template methods         ////////////
///////////////////////////////////////////////////////////////////////////////
template<typename _DataType, int _Dim_X>
Discrete<_DataType, _Dim_X>::Discrete(int reserve_size) {
  Trajectory<_DataType, _Dim_X>::range(std::numeric_limits<_DataType>::min(), std::numeric_limits<_DataType>::max());
  sorted_t_set_.reserve(reserve_size);
}

template<typename _DataType, int _Dim_X>
Discrete<_DataType, _Dim_X>::~Discrete() {
  clear();
}

template<typename _DataType, int _Dim_X>
void Discrete<_DataType, _Dim_X>::push_back(_DataType _t,
    const typename Trajectory<_DataType, _Dim_X>::StateVec& _s) {
  seqs_[_t] = _s;
  // seqs_.insert(std::make_pair(_t, _s));
  sorted_t_set_.push_back(_t);
  std::sort(sorted_t_set_.begin(), sorted_t_set_.end());
  if (Trajectory<_DataType, _Dim_X>::floor()   > _t)
      Trajectory<_DataType, _Dim_X>::range(_t, Trajectory<_DataType, _Dim_X>::ceiling());
  if (Trajectory<_DataType, _Dim_X>::ceiling() < _t)
    Trajectory<_DataType, _Dim_X>::range(Trajectory<_DataType, _Dim_X>::floor(), _t);
}

template<typename _DataType, int _Dim_X>
void Discrete<_DataType, _Dim_X>::clear() {
  seqs_.clear();
  sorted_t_set_.clear();
}

template<typename _DataType, int _Dim_X>
void Discrete<_DataType, _Dim_X>::save(const std::string& _fn) {
  std::ofstream _ofd(_fn);
  if (!_ofd.is_open()) {
    LOG_ERROR << "Open the file " << _fn << "fail!";
    return;
  }
  for (const auto& t : sorted_t_set_) {
    const auto& xt = seqs_[t];
    _ofd << t;
    for (Eigen::Index idx = 0; idx < xt.size(); ++idx)
      _ofd << "," << xt(idx);
    _ofd << std::endl;
  }
}

template<typename _DataType, int _Dim_X>
typename Trajectory<_DataType, _Dim_X>::StateVec
Discrete<_DataType, _Dim_X>::sample(_DataType _t) const {
  _DataType _min  = std::numeric_limits<_DataType>::min();
  _DataType _max  = std::numeric_limits<_DataType>::max();
  for (size_t i = 0; i < sorted_t_set_.size(); ++i) {
    if (sorted_t_set_[i] > _t) {
      _min = (0 == i) ? sorted_t_set_[i] : sorted_t_set_[i - 1];
      _max = sorted_t_set_[i];
      break;
    } else {
      _min = sorted_t_set_[i];
    }
  }
  ///! head or middle
  if (_max != std::numeric_limits<_DataType>::max()) {
    if (_max == _min) ///! head
      return seqs_.at(_min);

    double alpha = (_t - _min)/(_max - _min);
    return (seqs_.at(_min)*(1 - alpha) + seqs_.at(_max)*alpha);
  } else { ///! the end
    return seqs_.at(_min);
  }


  return (seqs_.end() == seqs_.find(_t)) ? typename Trajectory<_DataType, _Dim_X>::StateVec()
      : seqs_.at(_t);
//  for (const auto& t : t_set_) {
//    if ();
//  }
//  _t = __clamp(_t, coj_range_->floor, coj_range_->ceiling);
//
//  for (size_t i = 0; i < ranges_.size(); ++i)   {
//    if ((_t >= ranges_[i]->floor) && (_t <= ranges_[i]->ceiling)) {
//      return segs_[i]->sample(_t);
//    }
//  }
//
//  ///! Don't should be here.
//  assert(false && ("range error"));
//  return segs_[0]->sample(_t);
}

template<typename _T1, int _T2>
std::ostream& operator<<(std::ostream& os, const Discrete<_T1, _T2>& traj) {
//  for (const auto& t : traj.seqs_) {
//    os << "{" << t.first << ";\n" << t.second << "};\n";
//  }

  for (const auto& t : traj.sorted_t_set_) {
    os << t << " ";
  }
  return os;
}

#define DIS_TRAJ_MAKE_TYPEDEFS(Type, TypeSuffix, Size, SizeSuffix)   \
using DisTraj##SizeSuffix##TypeSuffix##Sp = boost::shared_ptr<Discrete<Type, Size>>; \
using DisTraj##SizeSuffix##TypeSuffix   = Discrete<Type, Size>;

#define DIS_TRAJ_MAKE_TYPEDEFS_ALL_SIZES(Type, TypeSuffix) \
    DIS_TRAJ_MAKE_TYPEDEFS(Type, TypeSuffix, 1, 1) \
    DIS_TRAJ_MAKE_TYPEDEFS(Type, TypeSuffix, 2, 2) \
    DIS_TRAJ_MAKE_TYPEDEFS(Type, TypeSuffix, 3, 3) \
    DIS_TRAJ_MAKE_TYPEDEFS(Type, TypeSuffix, 4, 4)

//DIS_TRAJ_MAKE_TYPEDEFS_ALL_SIZES(int,    i)
//DIS_TRAJ_MAKE_TYPEDEFS_ALL_SIZES(char,   c)
//DIS_TRAJ_MAKE_TYPEDEFS_ALL_SIZES(short,  s)
//DIS_TRAJ_MAKE_TYPEDEFS_ALL_SIZES(float,  f)
DIS_TRAJ_MAKE_TYPEDEFS_ALL_SIZES(double, d)

#undef SEG_TRAJ_MAKE_TYPEDEFS_ALL_SIZES
#undef SEG_TRAJ_MAKE_TYPEDEFS

} /* namespace qr_control */



#endif /* INCLUDE_ADT_DISCRETE_H_ */
