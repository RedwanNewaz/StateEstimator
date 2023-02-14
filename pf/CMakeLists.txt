

add_executable(pf_test main.cpp ../include/filters_common/helper.h particle_filter.cpp particle_filter.h)

target_link_libraries(pf_test
        Eigen3::Eigen
        )