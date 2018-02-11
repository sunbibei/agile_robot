#ifndef __EXPR_H_
#define __EXPR_H_

#include <string>
#include <map>

namespace internal {

bool eval(const std::string& str, double& out);

} /* namespace internal */

#endif
