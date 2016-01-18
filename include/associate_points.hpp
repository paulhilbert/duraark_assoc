#ifndef DURAARK_ASSOC_ASSOCIATE_POINTS_HPP_
#define DURAARK_ASSOC_ASSOCIATE_POINTS_HPP_

#include "common.hpp"

namespace duraark_assoc {

template <typename PointType, typename ColorType>
EXPORT_DECL std::vector<int32_t> associate_points(
    typename pcl::PointCloud<PointType>::ConstPtr cloud,
    const ifc_objects_t<ColorType> &objects, float epsilon,
    std::vector<uint32_t> &possible_hits, std::vector<uint32_t> &actual_hits,
    const Eigen::Vector3f &origin, const std::vector<uint32_t> &subset,
    const Eigen::Affine3f &t0 = Eigen::Affine3f::Identity(),
    const Eigen::Affine3f &t1 = Eigen::Affine3f::Identity(),
    const std::set<std::string> &ignore_entity_types = std::set<std::string>());

template <typename PointType, typename ColorType>
EXPORT_DECL std::vector<int32_t> associate_points(
    const ifc_objects_t<ColorType> &objects,
    typename pcl::PointCloud<PointType>::ConstPtr cloud, float epsilon,
    std::vector<uint32_t> &possible_hits, std::vector<uint32_t> &actual_hits,
    const Eigen::Vector3f &origin, const std::vector<uint32_t> &subset,
    const Eigen::Affine3f &t0 = Eigen::Affine3f::Identity(),
    const Eigen::Affine3f &t1 = Eigen::Affine3f::Identity(),
    const std::set<std::string> &ignore_entity_types = std::set<std::string>());

template <typename PointType, typename ColorType>
EXPORT_DECL std::vector<int32_t> associate_points(
    typename pcl::PointCloud<PointType>::ConstPtr cloud,
    const ifc_objects_t<ColorType> &objects, float epsilon,
    std::vector<uint32_t> &possible_hits, std::vector<uint32_t> &actual_hits,
    const std::vector<Eigen::Vector3f> &origins,
    const std::vector<uint32_t> &counts,
    const Eigen::Affine3f &t0 = Eigen::Affine3f::Identity(),
    const Eigen::Affine3f &t1 = Eigen::Affine3f::Identity(),
    const std::set<std::string> &door_entity_types = std::set<std::string>(),
    const std::set<std::string> &ignore_entity_types = std::set<std::string>());

template <typename PointType, typename ColorType>
EXPORT_DECL std::vector<int32_t> associate_points(
    const ifc_objects_t<ColorType> &objects,
    typename pcl::PointCloud<PointType>::ConstPtr cloud, float epsilon,
    std::vector<uint32_t> &possible_hits, std::vector<uint32_t> &actual_hits,
    const std::vector<Eigen::Vector3f> &origins,
    const std::vector<uint32_t> &counts,
    const Eigen::Affine3f &t0 = Eigen::Affine3f::Identity(),
    const Eigen::Affine3f &t1 = Eigen::Affine3f::Identity(),
    const std::set<std::string> &door_entity_types = std::set<std::string>(),
    const std::set<std::string> &ignore_entity_types = std::set<std::string>());

}  // duraark_assoc

#endif /* DURAARK_ASSOC_ASSOCIATE_POINTS_HPP_ */
