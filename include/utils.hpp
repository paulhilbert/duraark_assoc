#ifndef _DURAARK_ASSOC_UTILS_HPP_
#define _DURAARK_ASSOC_UTILS_HPP_

#include "common.hpp"

namespace duraark_assoc {

EXPORT_DECL std::map<uint32_t, std::vector<int32_t>>
to_object_map(const std::vector<int32_t> &associations,
              uint32_t min_point_count);

}  // duraark_assoc

#endif /* _DURAARK_ASSOC_UTILS_HPP_ */
