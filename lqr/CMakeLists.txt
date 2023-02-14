
add_executable(lqr main.cpp LQR.cpp LQR.h quadmodel.h)

target_link_libraries(lqr
        Eigen3::Eigen
        -lpython3.7m
        )