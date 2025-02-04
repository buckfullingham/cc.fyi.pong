find_package(Catch2 REQUIRED)
find_package(Eigen3 REQUIRED)

include(CTest)
include(Catch)

add_library(test-lib INTERFACE
)

target_link_libraries(test-lib INTERFACE
        Catch2::Catch2WithMain
        Eigen3::Eigen
)

target_include_directories(test-lib INTERFACE
        ../main
)

add_executable(geometry
        geometry.cpp
)

target_link_libraries(geometry PRIVATE
        test-lib
)

add_executable(model
        model.cpp
)

target_link_libraries(model PRIVATE
        test-lib
)

catch_discover_tests(geometry EXTRA_ARGS "--rng-seed=${PRNG_SEED}")
catch_discover_tests(model EXTRA_ARGS "--rng-seed=${PRNG_SEED}")
