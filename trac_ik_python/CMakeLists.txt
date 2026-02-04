cmake_minimum_required(VERSION 3.8)
project(trac_ik_python)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(trac_ik_lib REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(urdf REQUIRED)
find_package(SWIG REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(pkg_nlopt REQUIRED nlopt)

# Python 3
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)

# SWIG configuration
include(UseSWIG)
set(CMAKE_SWIG_FLAGS "-c++")

# Include directories
include_directories(
  ${trac_ik_lib_INCLUDE_DIRS}
  ${kdl_parser_INCLUDE_DIRS}
  ${urdf_INCLUDE_DIRS}
  ${pkg_nlopt_INCLUDE_DIRS}
  ${Python3_INCLUDE_DIRS}
)

# Set C compiler to C++ compiler for SWIG
set(CMAKE_C_COMPILER ${CMAKE_CXX_COMPILER})

# SWIG wrapper
set(CMAKE_SWIG_OUTDIR ${CMAKE_CURRENT_BINARY_DIR}/trac_ik_python)

swig_add_library(trac_ik_wrap
  TYPE SHARED
  LANGUAGE python
  SOURCES swig/trac_ik_wrap.i
)

# Set include directories for the generated C++ wrapper code
# This is needed because trac_ik.hpp includes other headers like <trac_ik/nlopt_ik.hpp>
# We need both the source directory (for relative path includes) and install directory
target_include_directories(_trac_ik_wrap PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/swig
  ${CMAKE_CURRENT_SOURCE_DIR}/../trac_ik_lib/include  # Source directory for trac_ik headers
  ${trac_ik_lib_INCLUDE_DIRS}  # Install directory
  ${kdl_parser_INCLUDE_DIRS}
  ${urdf_INCLUDE_DIRS}
  ${pkg_nlopt_INCLUDE_DIRS}
  ${Python3_INCLUDE_DIRS}
)


# Link libraries - use ament_target_dependencies for ROS2 packages
ament_target_dependencies(_trac_ik_wrap
  trac_ik_lib
  kdl_parser
  urdf
)

target_link_libraries(_trac_ik_wrap
  ${pkg_nlopt_LIBRARIES}
  ${Python3_LIBRARIES}
)

# Set output directory for Python module
set_target_properties(_trac_ik_wrap PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/trac_ik_python
  SWIG_MODULE_NAME trac_ik_wrap
)

# Install Python package
ament_python_install_package(${PROJECT_NAME}
  PACKAGE_DIR src/${PROJECT_NAME}
)

# Get Python install directory for SWIG files
ament_get_python_install_dir(python_install_dir)

# Install generated SWIG files to Python package directory
install(
  FILES
    ${CMAKE_CURRENT_BINARY_DIR}/trac_ik_python/trac_ik_wrap.py
  DESTINATION ${python_install_dir}/${PROJECT_NAME}
)

# Install shared library to Python package directory (same location as .py file)
install(
  TARGETS _trac_ik_wrap
  ARCHIVE DESTINATION ${python_install_dir}/${PROJECT_NAME}
  LIBRARY DESTINATION ${python_install_dir}/${PROJECT_NAME}
  RUNTIME DESTINATION ${python_install_dir}/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
