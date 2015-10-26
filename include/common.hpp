#ifndef _DURAARK_ASSOC_COMMON_HPP_
#define _DURAARK_ASSOC_COMMON_HPP_

#include <memory>
#include <vector>
#include <map>
#include <set>
#include <tuple>

#include <cereal/cereal.hpp>
#include <cereal/archives/json.hpp>
#include <cereal/types/vector.hpp>

#include <cartan/openmesh_traits.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#ifdef _WIN32
#define EXPORT_DECL __declspec(dllexport)
#else
#define EXPORT_DECL
#endif

namespace duraark_assoc {

struct object_info {
  std::string path;
  std::string guid;
  std::string type;

  template <typename Archive> void serialize(Archive &ar) {
    ar(CEREAL_NVP(path));
    ar(CEREAL_NVP(guid));
    ar(CEREAL_NVP(type));
  }
};

template <class ColorType>
using ifc_object_t =
    std::tuple<cartan::openmesh_t<ColorType>, std::string, std::string>;

template <class ColorType>
using ifc_objects_t = std::vector<ifc_object_t<ColorType>>;

} // duraark_assoc

#endif /* _DURAARK_ASSOC_COMMON_HPP_ */
