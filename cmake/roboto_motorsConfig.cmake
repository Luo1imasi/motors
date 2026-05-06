# RobotoMotorsConfig.cmake
#
# Config file for RobotoMotors
#
# Defines the IMPORTED target: roboto_motors::roboto_motors

# Check if we already have the target
if(TARGET roboto_motors::roboto_motors)
    return()
endif()

# Compute the installation prefix relative to this file
get_filename_component(CURRENT_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
# We install to ${PREFIX}/lib/cmake/RobotoMotors/, so we go up 3 levels to get PREFIX
get_filename_component(_INSTALL_PREFIX "${CURRENT_DIR}/../../../" ABSOLUTE)

set(RobotoMotors_INCLUDE_DIR "${_INSTALL_PREFIX}/include")
set(RobotoMotors_LIB_DIR     "${_INSTALL_PREFIX}/lib")

# Find dependencies
include(CMakeFindDependencyMacro)
find_dependency(fmt)
find_dependency(spdlog)
find_dependency(Eigen3)

# Helper function to find and add libraries
function(_add_imported_lib _lib_name)
    find_library(LIB_${_lib_name} 
        NAMES ${_lib_name}
        PATHS ${RobotoMotors_LIB_DIR}
        NO_DEFAULT_PATH
    )
    if(LIB_${_lib_name})
        list(APPEND RobotoMotors_LIBRARIES ${LIB_${_lib_name}})
        set(RobotoMotors_LIBRARIES ${RobotoMotors_LIBRARIES} PARENT_SCOPE)
    else()
        message(FATAL_ERROR "Could not find roboto_motors library: ${_lib_name} in ${RobotoMotors_LIB_DIR}")
    endif()
endfunction()

# Find all component libraries
set(RobotoMotors_LIBRARIES "")
_add_imported_lib(motors)

_add_imported_lib(dm_motors)
_add_imported_lib(evo_motors)
_add_imported_lib(lro_motors)
# _add_imported_lib(xyn_motors)

_add_imported_lib(motors_can)
_add_imported_lib(motors_canfd)
# _add_imported_lib(motors_ethercanfd)
# _add_imported_lib(motors_igh)
# _add_imported_lib(motors_soem)

# Create the INTERFACE target
add_library(roboto_motors::roboto_motors INTERFACE IMPORTED)

target_include_directories(roboto_motors::roboto_motors INTERFACE ${RobotoMotors_INCLUDE_DIR})
target_link_libraries(roboto_motors::roboto_motors INTERFACE 
    ${RobotoMotors_LIBRARIES}
    fmt::fmt
    spdlog::spdlog
    Eigen3::Eigen
)

message(STATUS "Found RobotoMotors: ${_INSTALL_PREFIX}")
