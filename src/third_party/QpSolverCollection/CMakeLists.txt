cmake_minimum_required(VERSION 3.24)
project(qp_solver_collection)

# Options for solvers
option(DEFAULT_ENABLE_ALL "Enable all QP solvers by default" OFF)
set(DEFAULT_ENABLE_VALUE ${DEFAULT_ENABLE_ALL})

option(ENABLE_QLD "Enable QLD" ${DEFAULT_ENABLE_VALUE})
option(ENABLE_QUADPROG "Enable QuadProg" ${DEFAULT_ENABLE_VALUE})
option(ENABLE_LSSOL "Enable LSSOL" ${DEFAULT_ENABLE_VALUE})
option(ENABLE_JRLQP "Enable JRLQP" ${DEFAULT_ENABLE_VALUE})
option(ENABLE_QPOASES "Enable qpOASES" ${DEFAULT_ENABLE_VALUE})
option(ENABLE_OSQP "Enable OSQP" ON)
option(ENABLE_NASOQ "Enable NASOQ" ${DEFAULT_ENABLE_VALUE})
option(ENABLE_HPIPM "Enable HPIPM" ${DEFAULT_ENABLE_VALUE})
option(ENABLE_PROXQP "Enable PROXQP" ${DEFAULT_ENABLE_VALUE})
option(ENABLE_QPMAD "Enable QPMAD" ${DEFAULT_ENABLE_VALUE})

option(FORCE_ALL_SOLVER_TEST "Test for all solvers regardless of options" OFF)
mark_as_advanced(FORCE FORCE_ALL_SOLVER_TEST)

option(SKIP_PRIVATE_SOLVER_TEST "Skip private solver test" OFF)
mark_as_advanced(FORCE SKIP_PRIVATE_SOLVER_TEST)

option(BUILD_SHARED_LIBS "Build libraries as shared as opposed to static" ON)
option(BUILD_TESTING "Build test" ON)
if(BUILD_TESTING)
  enable_testing()
endif()
include(GNUInstallDirs)

# To build shared libraries in Windows, we set CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS to TRUE.
# See https://cmake.org/cmake/help/v3.4/variable/CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS.html
# See https://blog.kitware.com/create-dlls-on-windows-without-declspec-using-new-cmake-export-all-feature/
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

# Eigen
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

set(EXIST_ENABLED_SOLVER FALSE)
macro(add_qp_solver _SOLVER_NAME)
  if(${ENABLE_${_SOLVER_NAME}})
    foreach(_PACKAGE_NAME ${ARGN})
      find_package(${_PACKAGE_NAME} REQUIRED)
    endforeach()
    set(EXIST_ENABLED_SOLVER TRUE)
  endif()
  message(NOTICE "- ENABLE_${_SOLVER_NAME}: ${ENABLE_${_SOLVER_NAME}}")
endmacro()

# eigen-qld
add_qp_solver(QLD eigen-qld)

# eigen-quadprog
add_qp_solver(QUADPROG eigen-quadprog)

# eigen-lssol
add_qp_solver(LSSOL eigen-lssol)

# JRLQP
add_qp_solver(JRLQP jrl-qp)

# qpOASES
add_qp_solver(QPOASES)

# Jazzy provides OSQP via osqp_vendor. OsqpEigen also supports a system OSQP.
if(ENABLE_OSQP)
  find_package(OsqpEigen 0.10.3 QUIET)
  if(NOT OsqpEigen_FOUND)
    find_package(osqp QUIET)
    if(NOT osqp_FOUND)
      find_package(osqp_vendor REQUIRED)
    endif()
    include(FetchContent)
    FetchContent_Declare(osqp_eigen
      URL https://github.com/robotology/osqp-eigen/archive/refs/tags/v0.10.3.tar.gz
      URL_HASH SHA256=1d9feff9b5f435e4596584aed1b923c58ee0c820d47566e35eb5ef7a56928d95
      DOWNLOAD_EXTRACT_TIMESTAMP TRUE
      TLS_VERIFY ON
    )
    FetchContent_MakeAvailable(osqp_eigen)
  endif()
  set(EXIST_ENABLED_SOLVER TRUE)
endif()

# NASOQ
add_qp_solver(NASOQ nasoq)

# HPIPM
add_qp_solver(HPIPM blasfeo hpipm)

# PROXQP
add_qp_solver(PROXQP proxsuite)

# QPMAD
add_qp_solver(QPMAD qpmad)

if(NOT ${EXIST_ENABLED_SOLVER})
  message(FATAL_ERROR "None of the QP solvers are enabled. Enable at least one QP solver.")
endif()

set(QP_SOLVER_OPTIONS_HEADER_FILE "${PROJECT_BINARY_DIR}/include/qp_solver_collection/QpSolverOptions.h")
configure_file(include/qp_solver_collection/QpSolverOptions.h.in "${QP_SOLVER_OPTIONS_HEADER_FILE}")

set(QP_SOLVER_OPTIONS_CMAKE_FILE "${PROJECT_BINARY_DIR}/qp_solver_options.cmake")
configure_file(cmake/qp_solver_options.cmake.in "${QP_SOLVER_OPTIONS_CMAKE_FILE}")

add_subdirectory(src)

install(EXPORT ${PROJECT_NAME}
  FILE ${PROJECT_NAME}Targets.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
  NAMESPACE ${PROJECT_NAME}::
  )

# Write initial config cmake file
file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/Config.cmake.in"
"
\@PACKAGE_INIT\@

include(CMakeFindDependencyMacro)
find_dependency(Eigen3)
include(\"\${CMAKE_CURRENT_LIST_DIR}/\@PROJECT_NAME\@Targets.cmake\")
include(\"\${CMAKE_CURRENT_LIST_DIR}/qp_solver_options.cmake\")

set(\@PROJECT_NAME\@_LIBRARIES \@PROJECT_NAME\@::QpSolverCollection)
")

include(CMakePackageConfigHelpers)
# generate the config file that is includes the exports
configure_package_config_file(${CMAKE_CURRENT_BINARY_DIR}/Config.cmake.in
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
  INSTALL_DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}"
  NO_SET_AND_CHECK_MACRO
  NO_CHECK_REQUIRED_COMPONENTS_MACRO
  )

# generate the version file for the config file
# Extract version numbers from package.xml
file(READ package.xml PACKAGE_XML)
string(REGEX MATCH "<version>[0-9]+\\.[0-9]+\\.[0-9]+</version>" DIRTY_VERSION_STRING ${PACKAGE_XML})
string(REGEX REPLACE "^<version>([0-9]+)\\.([0-9]+)\\.([0-9]+)</version>$" "\\1" QSC_MAJOR_VERSION "${DIRTY_VERSION_STRING}")
string(REGEX REPLACE "^<version>([0-9]+)\\.([0-9]+)\\.([0-9]+)</version>$" "\\2" QSC_MINOR_VERSION "${DIRTY_VERSION_STRING}")
string(REGEX REPLACE "^<version>([0-9]+)\\.([0-9]+)\\.([0-9]+)</version>$" "\\3" QSC_PATCH_VERSION "${DIRTY_VERSION_STRING}")
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
  VERSION "${QSC_MAJOR_VERSION}.${QSC_MINOR_VERSION}.${QSC_PATCH_VERSION}"
  COMPATIBILITY AnyNewerVersion
  )

# install config files
install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
  "${QP_SOLVER_OPTIONS_CMAKE_FILE}"
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
  )

if(BUILD_TESTING)
  add_subdirectory(tests)
endif()

OPTION(INSTALL_DOCUMENTATION "Generate and install the documentation" OFF)
if(INSTALL_DOCUMENTATION)
  add_subdirectory(doc)
endif()
