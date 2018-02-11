#ifndef __SIMPLE_TD_ESTIMATOR_H__
#define __SIMPLE_TD_ESTIMATOR_H__

#include "touchdown_estimator.h"

namespace middleware {

class SimpleTDEstimator : public TouchdownEstimator {
public:
	SimpleTDEstimator();
	virtual ~SimpleTDEstimator();

public:
	/**
	* @brief Adding the newest data into estimator. This method needs to implement
	*        by subclass.
	* @param _new_data The newest data
	*/
	virtual void input(double _new_data) override;
	/**
	* @brief This method has finished the evaluated process, and return its results.
	* @return Return true if touchdown, or return false.
	*/
	virtual bool eval() override;

private:
	double data[10] = { 0 };
public:

	double d[10] = { 0 };
	bool sw = 0;
};

} /* end namespace middleware */

#endif
