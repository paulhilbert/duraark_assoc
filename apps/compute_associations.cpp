#include <iostream>
#include <fstream>
#include <algorithm>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
namespace fs = boost::filesystem;
namespace po = boost::program_options;

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <pcl/common/transforms.h>
#include <pcl/search/octree.h>
#include <e57_pcl/read.hpp>
using e57_pcl::point_normal_t;
using e57_pcl::cloud_normal_t;

#include <duraark_rdf/turtle_input_helper.hpp>
#include <duraark_rdf/turtle_output_helper.hpp>
using namespace duraark_rdf;

#include <common.hpp>
#include <associate_points.hpp>
#include <utils.hpp>

char chartolower(char in){
    if(in<='Z' && in>='A')
        return in-('Z'-'z');
    return in;
}

cloud_normal_t::Ptr load_cloud(const fs::path& path, std::string& guid,
                               std::vector<Eigen::Vector3f>& scan_origins,
                               std::vector<uint32_t>& scan_sizes);
duraark_assoc::ifc_objects_t<OpenMesh::Vec4f> load_ifc(const fs::path& path, std::string& guid);

void
assoc_pc_to_pc(cloud_normal_t::Ptr cloud_a, const std::string& name_a,
               const std::string& guid_a, cloud_normal_t::Ptr cloud_b,
               const std::string& name_b, const std::string& guid_b,
               turtle_output& output, const Eigen::Matrix4f& reg, float eps) {
    pcl::transformPointCloud(*cloud_a, *cloud_a, reg);
    pcl::search::Octree<point_normal_t> octree(0.2);
    octree.setInputCloud(cloud_b);

    write_prologue(output);

    entity::sptr_t cloud_entity_a = std::make_shared<entity>(
        entity::type_t::PC, name_a, guid_a);
    entity::sptr_t cloud_entity_b = std::make_shared<entity>(
        entity::type_t::PC, name_b, guid_b);
    write_entity(output, *cloud_entity_a);
    write_entity(output, *cloud_entity_b);

    std::vector<int> all_indices;
    int index = 0;

    for (const auto& p : *cloud_a) {
        std::vector<int> indices(1);
        std::vector<float> dists(1);
        if (octree.nearestKSearch(p, 1, indices, dists) && dists[0] <= eps*eps) {
            all_indices.push_back(index);
        }
        ++index;
    }

    auto string_type = std::make_shared<literal_type>("xsd:string");
    auto integer_type = std::make_shared<literal_type>("xsd:nonNegativeInteger");
    auto indices_type = std::make_shared<literal_type>("types:indexSet");

    entity sub_ent(cloud_entity_a, all_indices, guid_b);

    write_entity(output, sub_ent);
    output.print_sentence(statement(uri(sub_ent.name()), uri("rel:pointSubsetOf"),
                                    uri(sub_ent.parent()->name())));
    output.print_sentence(statement(uri(sub_ent.name()),
                                    uri("rel:pointSubsetContains"),
                                    literal(sub_ent.indices_string(), indices_type)));
    output.print_sentence(
        statement(uri(sub_ent.name()), uri("rel:subsetRepOf"),
                  literal(sub_ent.representation_of(), string_type)));
}

void
assoc_pc_to_ifc(cloud_normal_t::Ptr cloud,
                const std::string& name_pc,
                const std::string& guid_pc,
                const std::vector<Eigen::Vector3f>& scan_origins,
                const std::vector<uint32_t>& scan_sizes,
                const duraark_assoc::ifc_objects_t<OpenMesh::Vec4f>& objects,
                turtle_output& output, const Eigen::Matrix4f& reg, float eps) {
    std::set<std::string> ignored_entity_types;
    ignored_entity_types.insert("ifcdoor");
    std::vector<uint32_t> possible(objects.size()), actual(objects.size());
    Eigen::Affine3f t0, t1 = Eigen::Affine3f::Identity();
    t0 = reg;
    auto assoc =
        duraark_assoc::associate_points<point_normal_t, OpenMesh::Vec4f>(
            objects, cloud, eps, possible, actual, scan_origins, scan_sizes, t0,
            t1, ignored_entity_types);

    entity::sptr_t cloud_entity = std::make_shared<entity>(
        entity::type_t::PC, name_pc, guid_pc);
    write_prologue(output);
    write_entity(output, *cloud_entity);
    auto obj_map = duraark_assoc::to_object_map(assoc, 1);
    for (const auto& m : obj_map) {
        entity sub_ent(cloud_entity, m.second, std::get<1>(objects[m.first]));
        write_subset_entity(output, sub_ent, possible[m.first]);
    }
}

int
main(int argc, char const* argv[]) {
    std::string file_a;
    std::string file_b;
    std::string file_reg;
    std::string file_out;
    float eps;

    po::options_description desc("compute_associations command line options");
    desc.add_options()("help,h", "Help message")
        ("rep-a,a", po::value<std::string>(&file_a)->required(), "First input representation file")
        ("rep-b,b", po::value<std::string>(&file_b)->required(), "Second input representation file")
        ("registration,r", po::value<std::string>(&file_reg)->default_value(""), "Optional .rdf registration file")
        ("output-file,o", po::value<std::string>(&file_out)->required(),"Output .rdf association file")
        ("epsilon,e", po::value<float>(&eps)->default_value(0.1f), "Distance Threshold")
    ;

    // Check for required options.
    po::variables_map vm;
    bool optionsException = false;
    try {
        po::store(po::command_line_parser(argc, argv).options(desc).run(), vm);
        // po::store(po::parse_command_line(argc, argv, desc), vm);
        po::notify(vm);
    } catch (std::exception& e) {
        if (!vm.count("help")) {
            std::cout << e.what() << "\n";
        }
        optionsException = true;
    }
    if (optionsException || vm.count("help")) {
        std::cout << desc << "\n";
        return optionsException ? 1 : 0;
    }

    fs::path path_a(file_a);
    fs::path path_b(file_b);
    fs::path path_out(file_out);

    if (!fs::exists(path_a)) {
        std::cerr << "File \"" << path_a.string()
                  << "\" does not exist."
                  << "\n";
        return 1;
    }
    if (!fs::exists(path_b)) {
        std::cerr << "File \"" << path_b.string()
                  << "\" does not exist."
                  << "\n";
        return 1;
    }
    if (file_reg != "") {
        fs::path path_reg(file_reg);
        if (!fs::exists(path_reg) || path_reg.extension() != ".rdf") {
            std::cerr << "File \"" << path_reg.string()
                      << "\" does not exist or has wrong extension (\".rdf\")."
                      << "\n";
            return 1;
        }
    }
    if (!fs::exists(path_out.parent_path())) {
        fs::create_directories(path_out.parent_path());
    }

    std::string ext_a = path_a.extension().string(), ext_b = path_b.extension().string();
    std::transform(ext_a.begin(), ext_a.end(), ext_a.begin(), &chartolower);
    std::transform(ext_b.begin(), ext_b.end(), ext_b.begin(), &chartolower);

    turtle_output output(path_out.string());
    Eigen::Matrix4f reg = Eigen::Matrix4f::Identity();

    if (ext_a == ".e57n" && ext_b == ".e57n") {
        std::string guid_a, guid_b;
        std::string name_a = path_a.stem().string();
        std::string name_b = path_b.stem().string();
        std::vector<Eigen::Vector3f> scan_origins;
        std::vector<uint32_t> scan_sizes;
        cloud_normal_t::Ptr cloud_a =
            load_cloud(path_a, guid_a, scan_origins, scan_sizes);
        cloud_normal_t::Ptr cloud_b =
            load_cloud(path_b, guid_b, scan_origins, scan_sizes);


        if (file_reg != "") {
            duraark_rdf::turtle_input input(file_reg);
            reg = duraark_rdf::parse_registration(input, guid_a, guid_b);
        }

        assoc_pc_to_pc(cloud_a, name_a, guid_a, cloud_b, name_b, guid_b, output,
                       reg, eps);
    } else if (ext_a == ".ifcmesh" && ext_b == ".e57n") {
        std::string guid_ifc, guid_pc;
        std::vector<Eigen::Vector3f> scan_origins;
        std::vector<uint32_t> scan_sizes;
        cloud_normal_t::Ptr cloud =
            load_cloud(path_b, guid_pc, scan_origins, scan_sizes);
        std::string name_pc = path_b.stem().string();
        duraark_assoc::ifc_objects_t<OpenMesh::Vec4f> objects = load_ifc(path_a, guid_ifc);

        if (file_reg != "") {
            duraark_rdf::turtle_input input(file_reg);
            reg = duraark_rdf::parse_registration(input, guid_ifc, guid_pc);
        }

        assoc_pc_to_ifc(cloud, name_pc, guid_pc, scan_origins, scan_sizes, objects, output,
                        reg, eps);
    } else if (ext_a == ".e57n" && ext_b == ".ifcmesh") {
        std::string guid_ifc, guid_pc;
        std::vector<Eigen::Vector3f> scan_origins;
        std::vector<uint32_t> scan_sizes;
        cloud_normal_t::Ptr cloud =
            load_cloud(path_a, guid_pc, scan_origins, scan_sizes);
        std::string name_pc = path_a.stem().string();
        duraark_assoc::ifc_objects_t<OpenMesh::Vec4f> objects = load_ifc(path_b, guid_ifc);

        if (file_reg != "") {
            duraark_rdf::turtle_input input(file_reg);
            reg = duraark_rdf::parse_registration(input, guid_pc, guid_ifc);
        }

        assoc_pc_to_ifc(cloud, name_pc, guid_pc, scan_origins, scan_sizes, objects, output,
                        reg, eps);
    } else {
        std::cerr << "Invalid file extensions. Supported are only (.ifcmesh -> .e57n) and (.e57n -> .e57n)" << "\n";
    }

}

cloud_normal_t::Ptr
load_cloud(const fs::path& path, std::string& guid,
           std::vector<Eigen::Vector3f>& scan_origins,
           std::vector<uint32_t>& scan_sizes) {
    auto clouds = e57_pcl::load_e57_scans_with_normals(path.string(), guid);
    cloud_normal_t::Ptr cloud(new cloud_normal_t());
    for (auto c : clouds) {
        cloud->insert(cloud->end(), c->begin(), c->end());
        scan_origins.push_back(c->sensor_origin_.head(3));
        scan_sizes.push_back(c->size());
    }
    return cloud;
}

duraark_assoc::ifc_objects_t<OpenMesh::Vec4f>
load_ifc(const fs::path& path, std::string& guid) {
    std::vector<duraark_assoc::object_info> objects;
    std::ifstream in(path.string().c_str());
    if (!in.good()) {
        throw std::runtime_error("Unable to open file \"" + path.string() +
                                 "\" for reading.");
    }
    {
        cereal::JSONInputArchive ar(in);
        ar(cereal::make_nvp("guid", guid));
        ar(cereal::make_nvp("objects", objects));
    }

    duraark_assoc::ifc_objects_t<OpenMesh::Vec4f> result;
    for (const auto& obj : objects) {
        fs::path mesh_path = path.parent_path() / obj.path;
        duraark_assoc::ifc_object_t<OpenMesh::Vec4f> ifc_obj;

        if (!cartan::mesh_traits<cartan::openmesh_t<OpenMesh::Vec4f>>::read(std::get<0>(ifc_obj), mesh_path.string())) {
            throw std::runtime_error("Could not open mesh file \"" +
                                     mesh_path.string() + "\"");
        }
        std::get<0>(ifc_obj).request_vertex_normals();
        std::get<0>(ifc_obj).request_face_normals();
        std::get<0>(ifc_obj).update_face_normals();
        std::get<0>(ifc_obj).update_normals();

        std::get<1>(ifc_obj) = obj.guid;
        std::get<2>(ifc_obj) = obj.type;

        result.push_back(ifc_obj);
    }

    return result;
}
