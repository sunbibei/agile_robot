/*
 * seg_trajectory.h
 *
 *  Created on: Jan 5, 2018
 *      Author: bibei
 */

#ifndef INCLUDE_ADT_SEGMENTED_H_
#define INCLUDE_ADT_SEGMENTED_H_

#include "trajectory.h"

namespace agile_control {

/*!
 * @brief The implement of Segmented trajectory.
 */
template<typename _DataType, int _Dim_X>
class Segmented : public Trajectory<_DataType, _Dim_X> {
public:
  /*!
   * @brief The default constructor.
   */
  Segmented();

  virtual ~Segmented();

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
  void add(const typename Trajectory<_DataType, _Dim_X>::TrajSp&, _DataType _from, _DataType _to);
  void add(const typename Trajectory<_DataType, _Dim_X>::TrajSp&);
  ///! This method remove a segmented trajectory
  // void remove(Trajectory<_DataType, _Dim_X>*);

//  ///! This method return the start point
//  _DataType floor(bool* exit = nullptr)   const;
//  ///! This method return the end   point
//  _DataType ceiling(bool* exit = nullptr) const;

public:
//  template<typename _T1, int _T2>
//  friend std::ostream& operator<<(std::ostream&, const Segmented<_T1, _T2>& traj);

protected:
  MiiVector<typename Trajectory<_DataType, _Dim_X>::TrajSp> seqs_;
  MiiVector<typename Trajectory<_DataType, _Dim_X>::Range*> ranges_;
  // typename Trajectory<_DataType, _Dim_X>::Range*            coj_range_;
};



///////////////////////////////////////////////////////////////////////////////
////////////        The implementation of template methods         ////////////
///////////////////////////////////////////////////////////////////////////////
const size_t DEFAULT_SEGS = 8;
template<typename _DataType, int _Dim_X>
Segmented<_DataType, _Dim_X>::Segmented() {
  seqs_.reserve(DEFAULT_SEGS);
  // for (auto& s : segs_)   s = nullptr;

  ranges_.reserve(DEFAULT_SEGS);
  // for (auto& r : ranges_) r = nullptr;
  Trajectory<_DataType, _Dim_X>::range_ = nullptr;
}

template<typename _DataType, int _Dim_X>
Segmented<_DataType, _Dim_X>::~Segmented() {
  clear();
}

template<typename _DataType, int _Dim_X>
void Segmented<_DataType, _Dim_X>::add(
    const typename Trajectory<_DataType, _Dim_X>::TrajSp& _new_traj) {
  bool is_range = false;
  _DataType floor   = _new_traj->floor(&is_range);
  _DataType ceiling = _new_traj->ceiling(&is_range);
  assert(is_range && ("The added trajectory is not set the range"));

  seqs_.push_back(_new_traj);

  auto tmp_range = new typename Trajectory<_DataType, _Dim_X>::Range;
  tmp_range->floor   = floor;
  tmp_range->ceiling = ceiling;
  ranges_.push_back(tmp_range);

  if (!Trajectory<_DataType, _Dim_X>::range_) {
    Trajectory<_DataType, _Dim_X>::range_ = new typename Trajectory<_DataType, _Dim_X>::Range;
    Trajectory<_DataType, _Dim_X>::range_->floor   = floor;
    Trajectory<_DataType, _Dim_X>::range_->ceiling = ceiling;
  } else {
    Trajectory<_DataType, _Dim_X>::range_->floor   = std::min(Trajectory<_DataType, _Dim_X>::range_->floor,   floor);
    Trajectory<_DataType, _Dim_X>::range_->ceiling = std::max(Trajectory<_DataType, _Dim_X>::range_->ceiling, ceiling);
  }
}

template<typename _DataType, int _Dim_X>
void Segmented<_DataType, _Dim_X>::add(
    const typename Trajectory<_DataType, _Dim_X>::TrajSp& _new_traj, _DataType _from, _DataType _to) {
  // auto tmp = new Trajectory<_DataType, _Dim_X>(_new_traj);
  seqs_.push_back(_new_traj);

  auto tmp_range = new typename Trajectory<_DataType, _Dim_X>::Range;
  tmp_range->floor   = _from;
  tmp_range->ceiling = _to;
  ranges_.push_back(tmp_range);

  if (!Trajectory<_DataType, _Dim_X>::range_) {
    Trajectory<_DataType, _Dim_X>::range_ = new typename Trajectory<_DataType, _Dim_X>::Range;
    Trajectory<_DataType, _Dim_X>::range_->floor   = _from;
    Trajectory<_DataType, _Dim_X>::range_->ceiling = _to;
  } else {
    Trajectory<_DataType, _Dim_X>::range_->floor   = std::min(Trajectory<_DataType, _Dim_X>::range_->floor,   _from);
    Trajectory<_DataType, _Dim_X>::range_->ceiling = std::max(Trajectory<_DataType, _Dim_X>::range_->ceiling, _to);
  }
}

//template<typename _DataType, int _Dim_X>
//void Segmented<_DataType, _Dim_X>::remove(Trajectory<_DataType, _Dim_X>* addr) {
//  for (size_t i = 0; i < segs_.size(); ++i)   {
//    if (segs_[i] == addr) {
//      delete segs_[i];
//      segs_[i] = nullptr;
//
//      delete ranges_[i];
//      ranges_[i] = nullptr;
//    }
//  }
//}

template<typename _DataType, int _Dim_X>
void Segmented<_DataType, _Dim_X>::clear() {
  for (auto& s : seqs_)   {
    s.reset();
  }
  seqs_.clear();

  for (auto& r : ranges_)   {
    delete r;
    r = nullptr;
  }
  ranges_.clear();

  delete Trajectory<_DataType, _Dim_X>::range_;
  Trajectory<_DataType, _Dim_X>::range_ = nullptr;
}

template<typename _DataType, int _Dim_X>
typename Trajectory<_DataType, _Dim_X>::StateVec
Segmented<_DataType, _Dim_X>::sample(_DataType _t) const {
  // assert(Trajectory<_DataType, _Dim_X>::range_ && "No trajectory be added!");
  _t = __clamp(_t, Trajectory<_DataType, _Dim_X>::range_->floor, Trajectory<_DataType, _Dim_X>::range_->ceiling);

  for (size_t i = 0; i < ranges_.size(); ++i)   {
    if ((_t >= ranges_[i]->floor) && (_t <= ranges_[i]->ceiling)) {
      return seqs_[i]->sample(_t);
    }
  }

  ///! Don't should be here.
  assert(false && ("range error"));
  return seqs_[0]->sample(_t);
}

//template<typename _DataType, int _Dim_X>
//_DataType Segmented<_DataType, _Dim_X>::floor(bool* exit) const {
//  if (exit) *exit = (nullptr != Trajectory<_DataType, _Dim_X>::range_);
//
//  return (Trajectory<_DataType, _Dim_X>::range_) ? Trajectory<_DataType, _Dim_X>::range_->floor : std::numeric_limits<_DataType>::min();
//}
//
//template<typename _DataType, int _Dim_X>
//_DataType Segmented<_DataType, _Dim_X>::ceiling(bool* exit) const {
//  if (exit) *exit = (nullptr != Trajectory<_DataType, _Dim_X>::range_);
//
//  return (Trajectory<_DataType, _Dim_X>::range_) ? Trajectory<_DataType, _Dim_X>::range_->ceiling : std::numeric_limits<_DataType>::max();
//}

//template<typename _T1, int _T2>
//std::ostream& operator<<(std::ostream& os, const Segmented<_T1, _T2>& traj) {
//  for (const auto& t : traj.segs_) {
//    os << *t;
//  }
//  return os;
//}

#define SEG_TRAJ_MAKE_TYPEDEFS(Type, TypeSuffix, Size, SizeSuffix)   \
using SegTraj##SizeSuffix##TypeSuffix##Sp = boost::shared_ptr<Segmented<Type, Size>>; \
using SegTraj##SizeSuffix##TypeSuffix     = Segmented<Type, Size>;

#define SEG_TRAJ_MAKE_TYPEDEFS_ALL_SIZES(Type, TypeSuffix) \
    SEG_TRAJ_MAKE_TYPEDEFS(Type, TypeSuffix, 1, 1) \
    SEG_TRAJ_MAKE_TYPEDEFS(Type, TypeSuffix, 2, 2) \
    SEG_TRAJ_MAKE_TYPEDEFS(Type, TypeSuffix, 3, 3) \
    SEG_TRAJ_MAKE_TYPEDEFS(Type, TypeSuffix, 4, 4) \

//SEG_TRAJ_MAKE_TYPEDEFS_ALL_SIZES(int,    i)
//SEG_TRAJ_MAKE_TYPEDEFS_ALL_SIZES(char,   c)
//SEG_TRAJ_MAKE_TYPEDEFS_ALL_SIZES(short,  s)
//SEG_TRAJ_MAKE_TYPEDEFS_ALL_SIZES(float,  f)
SEG_TRAJ_MAKE_TYPEDEFS_ALL_SIZES(double, d)

#undef SEG_TRAJ_MAKE_TYPEDEFS_ALL_SIZES
#undef SEG_TRAJ_MAKE_TYPEDEFS

} /* namespace qr_control */

#endif /* INCLUDE_ADT_SEGMENTED_H_ */
