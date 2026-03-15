cmake_minimum_required(VERSION 3.15)
project(marvin_sdk VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ─── pybind11 ────────────────────────────────────────────────────────────────
execute_process(
    COMMAND python3 -m pybind11 --cmakedir
    OUTPUT_VARIABLE PYBIND11_CMAKE_DIR
    OUTPUT_STRIP_TRAILING_WHITESPACE
)
find_package(pybind11 CONFIG REQUIRED HINTS ${PYBIND11_CMAKE_DIR})

# Disable LTO globally before pybind11_add_module to suppress ODR warnings
# caused by FxType.h defaulting to _FX_CPL_WIN_64_CASE_1_ cross-TU.
set(CMAKE_INTERPROCEDURAL_OPTIMIZATION OFF)

# ─── Sources ─────────────────────────────────────────────────────────────────
file(GLOB SDK_SRCS "src/*.cpp")

# ─── pybind11 extension module ───────────────────────────────────────────────
pybind11_add_module(marvin_sdk marvin_bind.cpp ${SDK_SRCS})

target_include_directories(marvin_sdk PRIVATE src)
target_compile_definitions(marvin_sdk PRIVATE CMPL_LIN)
target_compile_options(marvin_sdk     PRIVATE -Wall -w -O2)
target_link_libraries(marvin_sdk      PRIVATE pthread rt)

# ─── Install ─────────────────────────────────────────────────────────────────
# Detect Python site-packages at configure time
execute_process(
    COMMAND python3 -c "import sysconfig; print(sysconfig.get_path('platlib'))"
    OUTPUT_VARIABLE PYTHON_SITE_PACKAGES
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

install(TARGETS marvin_sdk
    LIBRARY DESTINATION ${PYTHON_SITE_PACKAGES}
)
