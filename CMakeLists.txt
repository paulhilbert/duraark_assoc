cmake_minimum_required(VERSION 2.8.8)
project(duraark_assoc)

set (duraark_assoc_VERSION_MAJOR 0)
set (duraark_assoc_VERSION_MINOR 1)

include_directories("${PROJECT_SOURCE_DIR}")
include_directories("${PROJECT_BINARY_DIR}")
include_directories("${PROJECT_SOURCE_DIR}/include")
set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake/modules)

if(CMAKE_COMPILER_IS_GNUCXX)
	add_definitions(-fPIC)
	add_definitions(-O3)
	add_definitions(-g)
	add_definitions(-std=c++1y)
    add_definitions(-Wall)
    add_definitions(-Wno-unused-function)
    add_definitions(-Wno-unused-local-typedefs)
    add_definitions(-Wno-deprecated-declarations)
endif()

find_package(PCL COMPONENTS common search octree)
find_package(Boost COMPONENTS program_options date_time iostreams serialization system filesystem regex thread)
find_package(OPTIX)
find_package(E57PCL)
find_package(DuraarkRDF)
find_package(OpenMesh)
find_package(Cartan)

file (GLOB_RECURSE obj RELATIVE "${PROJECT_SOURCE_DIR}" "src/*.cpp")
if (Boost_FOUND AND PCL_FOUND AND OPTIX_FOUND AND E57PCL_FOUND AND DURAARKRDF_FOUND AND CARTAN_FOUND)
	include_directories(${Boost_INCLUDE_DIRS})
	include_directories(${PCL_INCLUDE_DIRS})
	include_directories(${OPENMESH_INCLUDE_DIRS})
	include_directories(${OPTIX_INCLUDE_DIRS})
	include_directories(${E57PCL_INCLUDE_DIRS})
	include_directories(${DURAARKRDF_INCLUDE_DIRS})
	include_directories(${CARTAN_INCLUDE_DIRS})

    add_library(duraark_assoc SHARED ${obj})
    target_link_libraries(duraark_assoc ${Boost_LIBRARIES} ${PCL_LIBRARIES} ${OPTIX_LIBRARIES} ${E57PCL_LIBRARIES} ${DURAARKRDF_LIBRARIES} "dl")

    find_package(Boost COMPONENTS program_options date_time iostreams serialization system filesystem regex thread)
    add_executable(compute_associations "${PROJECT_SOURCE_DIR}/apps/compute_associations.cpp")
    target_link_libraries(compute_associations duraark_assoc ${Boost_LIBRARIES} ${PCL_LIBRARIES} ${OPENMESH_LIBRARIES} ${OPTIX_LIBRARIES} ${E57PCL_LIBRARIES} ${DURAARKRDF_LIBRARIES} ${CARTAN_LIBRARIES} "dl")

	# install binary
    install (TARGETS duraark_assoc DESTINATION lib)
    install (TARGETS compute_associations DESTINATION bin)
	# install header
    install (DIRECTORY include/ DESTINATION include/duraark_assoc)
    # install find module
    file (GLOB find_modules RELATIVE "${PROJECT_SOURCE_DIR}" "cmake/modules/self/*.cmake")
    install (FILES ${find_modules} DESTINATION share/DuraarkAssoc)
endif()
