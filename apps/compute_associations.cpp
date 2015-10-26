#include <iostream>
#include <fstream>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
namespace fs = boost::filesystem;
namespace po = boost::program_options;

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <e57_pcl/read.hpp>
using e57_pcl::point_normal_t;
using e57_pcl::cloud_normal_t;

#include <duraark_rdf/turtle_input_helper.hpp>
#include <duraark_rdf/turtle_output_helper.hpp>
using namespace duraark_rdf;

#include <common.hpp>
#include <associate_points.hpp>
#include <utils.hpp>

cloud_normal_t::Ptr load_cloud(const fs::path& path, std::string& guid,
                               std::vector<Eigen::Vector3f>& scan_origins,
                               std::vector<uint32_t>& scan_sizes);
duraark_assoc::ifc_objects_t<OpenMesh::Vec4f> load_ifc(const fs::path& path, std::string& guid);

int
main(int argc, char const* argv[]) {
    std::string file_ifc;
    std::string file_pc;
    std::string file_reg;
    std::string file_out;
    float eps;

    po::options_description desc("compute_associations command line options");
    desc.add_options()("help,h", "Help message")
        ("ifc_mesh,i", po::value<std::string>(&file_ifc)->required(), "Input .ifcmesh file")
        ("pointcloud,p", po::value<std::string>(&file_pc)->required(), "Input .e57n file")
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

    fs::path path_ifc(file_ifc);
    fs::path path_pc(file_pc);
    fs::path path_out(file_out);

    if (!fs::exists(path_ifc) || path_ifc.extension() != ".ifcmesh") {
        std::cerr << "File \"" << path_ifc.string()
                  << "\" does not exist or has wrong extension (\".ifcmesh\")."
                  << "\n";
        return 1;
    }
    if (!fs::exists(path_pc) || path_pc.extension() != ".e57n") {
        std::cerr << "File \"" << path_pc.string()
                  << "\" does not exist or has wrong extension (\".e57n\")."
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

    std::string guid_ifc, guid_pc;
    std::vector<Eigen::Vector3f> scan_origins;
    std::vector<uint32_t> scan_sizes;
    cloud_normal_t::Ptr cloud =
        load_cloud(path_pc, guid_pc, scan_origins, scan_sizes);
    duraark_assoc::ifc_objects_t<OpenMesh::Vec4f> objects = load_ifc(path_ifc, guid_ifc);

    Eigen::Matrix4f reg = Eigen::Matrix4f::Identity();
    if (file_reg != "") {
        duraark_rdf::turtle_input input(file_reg);
        reg = duraark_rdf::parse_registration(input, guid_ifc, guid_pc);
    }

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
        entity::type_t::PC, path_pc.stem().string(), guid_pc);
    turtle_output output(path_out.string());
    write_prologue(output);
    write_entity(output, *cloud_entity);
    auto obj_map = duraark_assoc::to_object_map(assoc, 1);
    for (const auto& m : obj_map) {
        entity sub_ent(cloud_entity, m.second, std::get<1>(objects[m.first]));
        write_subset_entity(output, sub_ent, possible[m.first]);
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
