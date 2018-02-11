/*
 * planar_motion_estimator.h
 *
 *  Created on: Sep 12, 2017
 *      Author: bibei
 */

#ifndef INCLUDE_REPOSITORY_ESTIMATOR_TOUCHDOWN_ESTIMATOR_H_
#define INCLUDE_REPOSITORY_ESTIMATOR_TOUCHDOWN_ESTIMATOR_H_

#include "foundation/label.h"
#include "foundation/utf.h"

namespace middleware {

class TouchdownEstimator : public Label {
public:
  /**
   * @brief Adding the newest data into estimator. This method needs to implement
   *        by subclass.
   * @param _new_data The newest data
   */
  virtual void input(double _new_data);
  /**
   * @brief This method has finished the evaluated process, and return its results.
   * @return Return true if touchdown, or return false.
   */
  virtual bool eval();

public:
  TouchdownEstimator(const MiiString& _l = Label::null);
  /**
   * @brief This method is proposed as a compromise for auto-instanceor.
   *        The all of the initialization will be completed in this method.
   *        You should get configure information from @MiiCfgReader, and fill
   *        the information into the configure file, for example
   *        <touchdown auto_inst="middleware::TouchdownEstimator"
   *                   param_1="xxx" param_2="xxx" />
   *        In the code, you could like this:
   *          auto cfg = MiiCfgReader::instance();
   *          cfg->get_value(getLabel(), "param_1", param_1_);
   *          cfg->get_value(getLabel(), "param_2", param_2_);
   */
  virtual bool auto_init() override;

  virtual ~TouchdownEstimator();
};

} /* namespace middleware */

#endif /* INCLUDE_REPOSITORY_ESTIMATOR_TOUCHDOWN_ESTIMATOR_H_ */
