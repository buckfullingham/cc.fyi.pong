find_package(Catch2 REQUIRED)

include(CTest)
include(Catch)

add_executable(tests
        tests.cpp
)

target_include_directories(tests PRIVATE
        ../main
)

target_link_libraries(tests PRIVATE
        Catch2::Catch2WithMain
)

add_test(NAME "breaking test" COMMAND node "breaking test")
