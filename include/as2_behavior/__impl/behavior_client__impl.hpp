#ifndef __AS2_BEHAVIOR_CLIENT__IMPL_HPP__
#define __AS2_BEHAVIOR_CLIENT__IMPL_HPP__
#include "as2_behavior/__detail/behavior_client__class.hpp"

namespace as2_behavior {

template <typename actionT>
std::string BehaviorClient<actionT>::generate_name(const std::string& name) {
  return std::string(this->get_name()) + "/_behavior/" + name;
}

};  // namespace as2_behavior

#endif  // AS2_BEHAVIOR__BEHAVIOR_SERVER_HPP_
