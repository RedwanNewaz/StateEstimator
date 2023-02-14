add_executable(ekf main.cpp extended_kalman_filter.cpp extended_kalman_filter.h ../include/filters_common/helper.h ../include/filters_common/logger.h)

target_link_libraries(ekf

        Eigen3::Eigen)