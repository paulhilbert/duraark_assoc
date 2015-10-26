###############################################################################
# Find DuraarkAssoc
#
# This sets the following variables:
# DURAARKASSOC_FOUND - True if DuraarkAssoc was found.
# DURAARKASSOC_INCLUDE_DIRS - Directories containing the DuraarkAssoc include files.
# DURAARKASSOC_LIBRARY_DIRS - Directories containing the DuraarkAssoc library.
# DURAARKASSOC_LIBRARIES - DuraarkAssoc library files.

if(WIN32)
    find_path(DURAARKASSOC_INCLUDE_DIR duraark_assoc PATHS "/usr/include" "/usr/local/include" "/usr/x86_64-w64-mingw32/include" "$ENV{PROGRAMFILES}" NO_DEFAULT_PATHS)

    find_library(DURAARKASSOC_LIBRARY_PATH duraark_assoc PATHS "/usr/lib" "/usr/local/lib" "/usr/x86_64-w64-mingw32/lib" NO_DEFAULT_PATHS)

    if(EXISTS ${DURAARKASSOC_LIBRARY_PATH})
        get_filename_component(DURAARKASSOC_LIBRARY ${DURAARKASSOC_LIBRARY_PATH} NAME)
        find_path(DURAARKASSOC_LIBRARY_DIR ${DURAARKASSOC_LIBRARY} PATHS "/usr/lib" "/usr/local/lib" "/usr/x86_64-w64-mingw32/lib" NO_DEFAULT_PATHS)
    endif()
else(WIN32)
    find_path(DURAARKASSOC_INCLUDE_DIR duraark_assoc PATHS "/usr/include" "/usr/local/include" "$ENV{PROGRAMFILES}" NO_DEFAULT_PATHS)
    find_library(DURAARKASSOC_LIBRARY_PATH duraark_assoc PATHS "/usr/lib" "/usr/local/lib" NO_DEFAULT_PATHS)

    if(EXISTS ${DURAARKASSOC_LIBRARY_PATH})
        get_filename_component(DURAARKASSOC_LIBRARY ${DURAARKASSOC_LIBRARY_PATH} NAME)
        find_path(DURAARKASSOC_LIBRARY_DIR ${DURAARKASSOC_LIBRARY} PATHS "/usr/lib" "/usr/local/lib" NO_DEFAULT_PATHS)
    endif()
endif(WIN32)

set(DURAARKASSOC_INCLUDE_DIRS ${DURAARKASSOC_INCLUDE_DIR})
set(DURAARKASSOC_LIBRARY_DIRS ${DURAARKASSOC_LIBRARY_DIR})
set(DURAARKASSOC_LIBRARIES ${DURAARKASSOC_LIBRARY})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(DuraarkAssoc DEFAULT_MSG DURAARKASSOC_INCLUDE_DIR DURAARKASSOC_LIBRARY DURAARKASSOC_LIBRARY_DIR)

mark_as_advanced(DURAARKASSOC_INCLUDE_DIR)
mark_as_advanced(DURAARKASSOC_LIBRARY_DIR)
mark_as_advanced(DURAARKASSOC_LIBRARY)
mark_as_advanced(DURAARKASSOC_LIBRARY_PATH)
