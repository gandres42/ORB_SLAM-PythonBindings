cmake_minimum_required(VERSION 3.5)
project(ORB_SLAM3_PYTHON_BINDING)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

# -march=native must match the ORB_SLAM3 build: its public headers pass Eigen /
# Sophus types by value, and Eigen's alignment depends on the enabled SIMD set.
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native")

# ORB_SLAM3 selects std::chrono::steady_clock when COMPILEDWITHC11 is defined.
add_definitions(-DCOMPILEDWITHC11)

# C++17 is required. These bindings include ORB_SLAM3's System.h, which pulls in
# Pangolin (via Viewer.h); recent Pangolin needs C++17.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
message(STATUS "Using C++${CMAKE_CXX_STANDARD}.")

# Python 3 (interpreter + development headers + NumPy headers).
find_package(Python3 COMPONENTS Interpreter Development NumPy REQUIRED)
# Boost.Python must match the Python version (e.g. python314 for Python 3.14).
find_package(Boost 1.45.0 REQUIRED COMPONENTS python${Python3_VERSION_MAJOR}${Python3_VERSION_MINOR})
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
# ORB_SLAM3's `make install` ships a CMake package (headers, libORB_SLAM3.so and
# the bundled DBoW2 / g2o / Sophus). If it was installed somewhere other than a
# default prefix, pass -DORB_SLAM3_DIR=<prefix>/lib/cmake/ORB_SLAM3.
find_package(ORB_SLAM3 REQUIRED CONFIG)

# RPATH fiddling for the library, so that the installed version can find the linked libraries
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# ORB_SLAM3_INCLUDE_DIR is <prefix>/include/ORB_SLAM3; its parent lets the
# sources include the headers as <ORB_SLAM3/System.h>.
get_filename_component(ORB_SLAM3_INCLUDE_ROOT "${ORB_SLAM3_INCLUDE_DIR}" DIRECTORY)

include_directories(
${PROJECT_SOURCE_DIR}/src
${ORB_SLAM3_INCLUDE_ROOT}
${EIGEN3_INCLUDE_DIR}
${Boost_INCLUDE_DIRS}
${Python3_INCLUDE_DIRS}
${Python3_NumPy_INCLUDE_DIRS}
)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

set(TARGET_MODULE_NAME orbslam3)
add_library(${TARGET_MODULE_NAME} SHARED
src/ORBSlamPython.cpp
src/pyboost_cv2_converter.cpp
src/pyboost_cv3_converter.cpp)
set_target_properties(${TARGET_MODULE_NAME} PROPERTIES PREFIX "")

# ORB_SLAM3::ORB_SLAM3 also carries its include directories and its DBoW2, g2o,
# OpenCV, Eigen and Pangolin dependencies.
target_link_libraries(${TARGET_MODULE_NAME}
${ORB_SLAM3_LIBRARIES}
${OpenCV_LIBS}
${Boost_LIBRARIES}
${Python3_LIBRARIES}
)

# Install into the directory the interpreter actually imports from. This is
# distribution-dependent (Debian/Ubuntu use "dist-packages", most others
# "site-packages"), so ask Python rather than guessing the folder name.
# Override with -DPYTHON_SITE_PACKAGES=... if you want a different location.
if(NOT PYTHON_SITE_PACKAGES)
  execute_process(
    COMMAND "${Python3_EXECUTABLE}" -c "import sysconfig; print(sysconfig.get_path('platlib'))"
    OUTPUT_VARIABLE PYTHON_SITE_PACKAGES
    OUTPUT_STRIP_TRAILING_WHITESPACE)
endif()
message(STATUS "Installing orbslam3 module to: ${PYTHON_SITE_PACKAGES}")
install(TARGETS ${TARGET_MODULE_NAME} DESTINATION "${PYTHON_SITE_PACKAGES}")
# Ship hand-written type stubs next to the compiled module so editors / type
# checkers (Pylance, Pyright, ty) resolve the API. A .pyi adjacent to the .so is
# the standard discovery location for a C-extension's stubs.
install(FILES ${PROJECT_SOURCE_DIR}/stubs/orbslam3.pyi DESTINATION "${PYTHON_SITE_PACKAGES}")
