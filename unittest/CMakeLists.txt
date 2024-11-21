

# ---- Dependencies ----

include(../cmake/CPM.cmake)


CPMAddPackage(
        NAME googletest
        GITHUB_REPOSITORY google/googletest
        GIT_TAG release-1.12.1
        VERSION 1.12.1
        OPTIONS "INSTALL_GTEST OFF" "gtest_force_shared_crt"
)

# ---- Create binary ----

add_executable(CollisionCheckerTest test_collision_checker.cpp)
target_link_libraries(CollisionCheckerTest gtest gtest_main gmock)
target_compile_features(CollisionCheckerTest PRIVATE cxx_std_17)

# ---- Enable testing ----

enable_testing()
add_test(CollisionCheckerTest CollisionCheckerTest)