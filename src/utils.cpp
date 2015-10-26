#include <utils.hpp>

namespace duraark_assoc {

std::map<uint32_t, std::vector<int32_t>>
to_object_map(const std::vector<int32_t>& associations,
              uint32_t min_point_count) {
    std::map<uint32_t, std::vector<int32_t>> result;

    for (uint32_t i = 0; i < associations.size(); ++i) {
        int32_t idx = associations[i];
        if (idx < 0) continue;
        uint32_t obj = static_cast<uint32_t>(idx);
        if (result.find(obj) == result.end()) {
            result[obj] = std::vector<int32_t>();
        }
        result[obj].push_back(static_cast<int32_t>(i));
    }
    for (auto it = result.begin(); it != result.end();) {
        if (it->second.size() < min_point_count) {
            it = result.erase(it);
        } else {
            ++it;
        }
    }
    return result;
}

}  // duraark_assoc
