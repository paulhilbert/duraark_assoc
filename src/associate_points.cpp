#include <associate_points.hpp>

#include <optix_prime/optix_primepp.h>
#include <pcl/octree/octree_search.h>
#include <primitive_detection/PrimitiveDetector.h>
#include <pcl/common/pca.h>

#define OPTIX_CONTEXT_TYPE RTP_CONTEXT_TYPE_CUDA  // RTP_CONTEXT_TYPE_CPU
#define OPTIX_BUF_TYPE RTP_BUFFER_TYPE_HOST       // RTP_BUFFER_TYPE_CUDA_LINEAR

namespace duraark_assoc {

template <typename PointType, typename ColorType>
void
associate_doors(const ifc_objects_t<ColorType> &objects,
               typename pcl::PointCloud<PointType>::ConstPtr cloud,
               std::vector<uint32_t> &possible_hits, std::vector<uint32_t> &actual_hits,
               std::vector<int32_t> &associations,
               const Eigen::Affine3f &t0,
               const Eigen::Affine3f &t1,
               const std::set<std::string>& door_entity_types,
               const std::set<std::string>& ignore_entity_types);

template <typename PointType, typename ColorType>
std::vector<int32_t>
associate_points(typename pcl::PointCloud<PointType>::ConstPtr cloud,
                 const ifc_objects_t<ColorType>& objects,
                 float epsilon, std::vector<uint32_t>& possible_hits,
                 std::vector<uint32_t>& actual_hits,
                 const Eigen::Vector3f& origin,
                 const std::vector<uint32_t>& subset, const Eigen::Affine3f& t0,
                 const Eigen::Affine3f& t1,
                 const std::set<std::string>& ignore_entity_types) {
    if (!subset.size() || !objects.size()) {
        return std::vector<int32_t>();
    }

    std::vector<float> triangles;
    std::vector<int32_t> object_map;

    int32_t object_idx = 0, triangle_idx = 0;
    for (const auto& object : objects) {
        if (ignore_entity_types.count(std::get<2>(object))) {
            ++object_idx;
            continue;
        }
        uint32_t num_faces = std::get<0>(object).n_faces();
        std::vector<int32_t> tmp(num_faces, object_idx++);
        object_map.insert(object_map.end(), tmp.begin(), tmp.end());
        triangles.resize(triangles.size() + num_faces * 9);
        for (auto it = std::get<0>(object).faces_begin();
             it != std::get<0>(object).faces_end(); ++it) {
            for (auto fvIt = std::get<0>(object).cfv_iter(*it); fvIt.is_valid();
                 ++fvIt) {
                auto pnt = std::get<0>(object).point(*fvIt);
                Eigen::Vector3f eigen_pnt(pnt[0], pnt[1], pnt[2]);
                eigen_pnt = t1 * eigen_pnt;
                triangles[triangle_idx++] = eigen_pnt[0];
                triangles[triangle_idx++] = eigen_pnt[1];
                triangles[triangle_idx++] = eigen_pnt[2];
            }
        }
    }

    optix::prime::Context context;
    try {
        context = optix::prime::Context::create(RTP_CONTEXT_TYPE_CUDA);
    } catch (optix::prime::Exception& e) {
        std::cout << "Non-working CUDA optix context. Defaulting to CPU."
                  << "\n";
        context = optix::prime::Context::create(RTP_CONTEXT_TYPE_CPU);
    }
    optix::prime::Model model = context->createModel();
    model->setTriangles(triangles.size() / 9, OPTIX_BUF_TYPE, triangles.data());
    model->update(0);
    model->finish();

    float* rays = new float[subset.size() * 8];
    int* hits = new int[subset.size() * 4];

    Eigen::Vector3f t_origin = t0 * origin;
    for (uint32_t s = 0; s < subset.size(); ++s) {
        uint32_t i = subset[s];
        Eigen::Vector3f dir =
            (t0 * cloud->points[i].getVector3fMap() - t_origin).normalized();
        for (uint32_t j = 0; j < 3; ++j) {
            rays[s * 8 + j] = t_origin[j];
            rays[s * 8 + 4 + j] = dir[j];
        }
        rays[s * 8 + 3] = 0.f;
        rays[s * 8 + 7] = 1000.f;
    }

    try {
        optix::prime::Query query = model->createQuery(RTP_QUERY_TYPE_CLOSEST);
        query->setRays(subset.size(),
                       RTP_BUFFER_FORMAT_RAY_ORIGIN_TMIN_DIRECTION_TMAX,
                       OPTIX_BUF_TYPE, rays);
        query->setHits(subset.size(), RTP_BUFFER_FORMAT_HIT_T_TRIID_U_V,
                       OPTIX_BUF_TYPE, hits);
        query->execute(0);
    } catch (optix::prime::Exception& e) {
        delete[] rays;
        delete[] hits;
        throw std::runtime_error("Prime Runtime Exception: " +
                                 e.getErrorString());
    }

    std::vector<int32_t> associations(subset.size(), -1);
    // std::set<int> debug_hit;
    for (uint32_t s = 0; s < subset.size(); ++s) {
        int hit = hits[4 * s + 1];
        // debug_hit.insert(hit);
        if (hit < 0) continue;
        possible_hits[object_map[hit]] += 1;
        float lambda = *reinterpret_cast<float*>(&hits[4 * s]);
        // compute normal
        Eigen::Vector3f dir(&(rays[s * 8 + 4]));
        int off = 9 * hit;
        Eigen::Vector3f p0(triangles[off + 0], triangles[off + 1],
                           triangles[off + 2]),
            p1(triangles[off + 3], triangles[off + 4], triangles[off + 5]),
            p2(triangles[off + 6], triangles[off + 7], triangles[off + 8]);
        Eigen::Vector3f normal = (p1 - p0).cross(p2 - p0).normalized();
        float adjustment = fabs(normal.dot(dir));
        float adjusted_epsilon = epsilon;
        if (adjustment > Eigen::NumTraits<float>::dummy_precision())
            adjusted_epsilon = adjusted_epsilon / adjustment;

        if (fabs(lambda -
                 (t0 * cloud->points[subset[s]].getVector3fMap() - t_origin)
                     .norm()) < adjusted_epsilon) {
            actual_hits[object_map[hit]] += 1;
            int32_t object = object_map[hit];
            associations[s] = object;
        }
    }

    delete[] rays;
    delete[] hits;

    return associations;
}

template <typename PointType, typename ColorType>
std::vector<int32_t>
associate_points(const ifc_objects_t<ColorType>& objects,
                 typename pcl::PointCloud<PointType>::ConstPtr cloud,
                 float epsilon, std::vector<uint32_t>& possible_hits,
                 std::vector<uint32_t>& actual_hits,
                 const Eigen::Vector3f& origin,
                 const std::vector<uint32_t>& subset, const Eigen::Affine3f& t0,
                 const Eigen::Affine3f& t1,
                 const std::set<std::string>& ignore_entity_types) {
    return associate_points<PointType, ColorType>(
        cloud, objects, epsilon, possible_hits, actual_hits, origin, subset, t1,
        t0, ignore_entity_types);
}

template <typename PointType, typename ColorType>
std::vector<int32_t>
associate_points(typename pcl::PointCloud<PointType>::ConstPtr cloud,
                 const ifc_objects_t<ColorType>& objects,
                 float epsilon, std::vector<uint32_t>& possible_hits,
                 std::vector<uint32_t>& actual_hits,
                 const std::vector<Eigen::Vector3f>& origins,
                 const std::vector<uint32_t>& counts, const Eigen::Affine3f& t0,
                 const Eigen::Affine3f& t1,
                 const std::set<std::string>& door_entity_types,
                 const std::set<std::string>& ignore_entity_types) {
    // The ignored entity types for point association is the union of ignored and door entity types.
    std::set<std::string> all_ignore_entity_types = ignore_entity_types;
    all_ignore_entity_types.insert(door_entity_types.begin(), door_entity_types.end());

    uint32_t begin = 0, end, count;
    std::vector<int32_t> associations;
    for (uint32_t i = 0; i < counts.size(); ++i) {
        count = counts[i];
        end = begin + count;
        std::vector<uint32_t> subset(end - begin);
        std::iota(subset.begin(), subset.end(), begin);
        auto assoc = associate_points<PointType, ColorType>(
            cloud, objects, epsilon, possible_hits, actual_hits, origins[i],
            subset, t0, t1, all_ignore_entity_types);
        associations.insert(associations.end(), assoc.begin(), assoc.end());
        begin += count;
    }

    if (door_entity_types.size()) {
        associate_doors<PointType, ColorType>(
            objects, cloud, possible_hits, actual_hits,
            associations, t1, t0, door_entity_types, ignore_entity_types
        );
    }

    /*
    // Debug: Only keep associations to doors.
    for (uint32_t i = 0; i < associations.size(); ++i) {
        if (associations[i] >= 0 && !door_entity_types.count(std::get<2>(objects[associations[i]]))) {
            associations[i] = -1;
        }
    }
    */

    return associations;
}

template <typename PointType, typename ColorType>
std::vector<int32_t>
associate_points(const ifc_objects_t<ColorType>& objects,
                 typename pcl::PointCloud<PointType>::ConstPtr cloud,
                 float epsilon, std::vector<uint32_t>& possible_hits,
                 std::vector<uint32_t>& actual_hits,
                 const std::vector<Eigen::Vector3f>& origins,
                 const std::vector<uint32_t>& counts, const Eigen::Affine3f& t0,
                 const Eigen::Affine3f& t1,
                 const std::set<std::string>& door_entity_types,
                 const std::set<std::string>& ignore_entity_types) {
    return associate_points<PointType, ColorType>(
        cloud, objects, epsilon, possible_hits, actual_hits, origins, counts,
        t1, t0, door_entity_types, ignore_entity_types);
}

template <typename PointType, typename ColorType>
void
associate_doors(const ifc_objects_t<ColorType> &objects,
               typename pcl::PointCloud<PointType>::ConstPtr cloud,
               std::vector<uint32_t> &possible_hits, std::vector<uint32_t> &actual_hits,
               std::vector<int32_t> &associations,
               const Eigen::Affine3f &t0,
               const Eigen::Affine3f &t1,
               const std::set<std::string>& door_entity_types,
               const std::set<std::string>& ignore_entity_types)
{
    pcl::octree::OctreePointCloudSearch<PointType> search_tree(0.5f);
    search_tree.setInputCloud(cloud);
    search_tree.addPointsFromInputCloud();

    pcshapes::PrimitiveDetector prim_detector;
    prim_detector.setEpsilon(0.01);
    prim_detector.setBitmapEpsilon(0.2);
    prim_detector.setNormalThreshold(0.98);
    prim_detector.setMinimumSupport(1000);
    prim_detector.setProbability(0.001);

    std::bitset<3> prim_types;
    prim_types.set(pcshapes::PLANE);

    int32_t object_idx = 0;
    for (const auto& object : objects) {
        // Ignore if object is not a door.
        if (!door_entity_types.count(std::get<2>(object))) {
            ++object_idx;
            continue;
        }

        // We work in the point cloud coordinate system since we need to perform a box search later.
        Eigen::Affine3f to_pc_trafo = t1.inverse() * t0;

        // Collect vertices of the door object.
        pcl::PointCloud<pcl::PointXYZ> object_vertices;
        for (auto it = std::get<0>(object).vertices_begin(); it != std::get<0>(object).vertices_end(); ++it) {
            auto pnt = std::get<0>(object).point(*it);
            Eigen::Vector3f eigen_pnt(pnt[0], pnt[1], pnt[2]);
            eigen_pnt = to_pc_trafo * eigen_pnt;
            pcl::PointXYZ vertex;
            vertex.getVector3fMap() = eigen_pnt;
            object_vertices.push_back(vertex);
        }

        // Compute axis-aligned object bbox.
        Eigen::AlignedBox<float, 3> object_bbox;
        for (const auto& v : object_vertices.points) {
            object_bbox.extend(v.getVector3fMap());
        }

        // Estimate local coordinate system of door.
        pcl::PCA<pcl::PointXYZ> pca(object_vertices);
        Eigen::Vector3f door_center = object_bbox.center();
        Eigen::Vector3f door_normal = Eigen::Vector3f(pca.getEigenVectors()(0, 2), pca.getEigenVectors()(1, 2), 0.f).normalized();
        Eigen::Vector3f door_left = Eigen::Vector3f(-door_normal[1], door_normal[0], 0.f);

        // Estimate width, depth, height of door.
        Eigen::AlignedBox<float, 3> local_object_bbox;
        for (const auto& v : object_vertices.points) {
            Eigen::Vector3f tmp = v.getVector3fMap() - door_center;
            local_object_bbox.extend(Eigen::Vector3f(tmp.dot(door_left), tmp.dot(door_normal), tmp[2]));
        }

        // Fetch point cloud subset around door.
        std::vector<int> nearby_point_indices;
        Eigen::Vector3f bbox_offset(local_object_bbox.sizes().head(2).maxCoeff(), local_object_bbox.sizes().head(2).maxCoeff(), 0.1f);
        search_tree.boxSearch(object_bbox.min() - bbox_offset, object_bbox.max() + bbox_offset, nearby_point_indices);

        /*
        for (auto i : nearby_point_indices) {
            associations[i] = object_idx;
        }
        */

        // Give up if too few points are found around door.
        if (nearby_point_indices.size() < prim_detector.getMinimumSupport()) {
            ++object_idx;
            continue;
        }

        // Detect and analyze plane primitives in point cloud subset.
        auto prims = prim_detector.detectPrimitives<PointType>(cloud, prim_types, false, nearby_point_indices);
        for (const auto& prim : prims) {
            auto plane = std::dynamic_pointer_cast<pcshapes::PrimitivePlane>(prim);
            Eigen::Vector3f local_plane_normal = t1.linear() * plane->normal();
            // Ignore if not approximately vertical.
            if (fabs(local_plane_normal[2]) > 0.05f) continue;
            // Ignore if parallel to the (closed) door.
            if (fabs(local_plane_normal.dot(door_left)) < 0.1f) continue;
            for (int point_index : plane->indices()) {
                // Ignore if point is already associated to some other object.
                if (associations[point_index] >= 0) continue;
                associations[point_index] = object_idx;
            }
        }

        ++object_idx;
    }
}

}  // duraark_assoc

#define INSTANTIATE_COLOR_POINT(point_type, color_type)                    \
    template std::vector<int32_t>                                          \
    duraark_assoc::associate_points<point_type, color_type>(               \
        typename pcl::PointCloud<point_type>::ConstPtr,                    \
        const ifc_objects_t<color_type>&, float,         \
        std::vector<uint32_t>&, std::vector<uint32_t>&,                    \
        const Eigen::Vector3f&, const std::vector<uint32_t>&,              \
        const Eigen::Affine3f&, const Eigen::Affine3f&,                    \
        const std::set<std::string>& ignore_entity_types);                 \
    template std::vector<int32_t>                                          \
    duraark_assoc::associate_points<point_type, color_type>(               \
        const ifc_objects_t<color_type>&,                \
        typename pcl::PointCloud<point_type>::ConstPtr, float,             \
        std::vector<uint32_t>&, std::vector<uint32_t>&,                    \
        const Eigen::Vector3f&, const std::vector<uint32_t>&,              \
        const Eigen::Affine3f&, const Eigen::Affine3f&,                    \
        const std::set<std::string>& ignore_entity_types);                 \
    template std::vector<int32_t>                                          \
    duraark_assoc::associate_points<point_type, color_type>(               \
        typename pcl::PointCloud<point_type>::ConstPtr,                    \
        const ifc_objects_t<color_type>&, float,         \
        std::vector<uint32_t>&, std::vector<uint32_t>&,                    \
        const std::vector<Eigen::Vector3f>&, const std::vector<uint32_t>&, \
        const Eigen::Affine3f&, const Eigen::Affine3f&,                    \
        const std::set<std::string>& door_entity_types,                    \
        const std::set<std::string>& ignore_entity_types);                 \
    template std::vector<int32_t>                                          \
    duraark_assoc::associate_points<point_type, color_type>(               \
        const ifc_objects_t<color_type>&,                \
        typename pcl::PointCloud<point_type>::ConstPtr, float,             \
        std::vector<uint32_t>&, std::vector<uint32_t>&,                    \
        const std::vector<Eigen::Vector3f>&, const std::vector<uint32_t>&, \
        const Eigen::Affine3f&, const Eigen::Affine3f&,                    \
        const std::set<std::string>& door_entity_types,                    \
        const std::set<std::string>& ignore_entity_types);
#include <pairwise_color_points.def>
