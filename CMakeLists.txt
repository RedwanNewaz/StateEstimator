cmake_minimum_required(VERSION 3.15)
project(StateEstimator)

set(CMAKE_CXX_STANDARD 17)

#find_package(Eigen3 REQUIRED)


include_directories(include)
include(FetchContent)
FetchContent_Declare(
        Eigen
        GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
        GIT_TAG master
        GIT_SHALLOW TRUE
        GIT_PROGRESS TRUE)
set(EIGEN_BUILD_DOC OFF)
# note: To disable eigen tests,
# you should put this code in a add_subdirectory to avoid to change
# BUILD_TESTING for your own project too since variables are directory
# scoped
set(BUILD_TESTING OFF)
set(EIGEN_BUILD_PKGCONFIG OFF)
set( OFF)
FetchContent_MakeAvailable(Eigen)
link_directories(${Python3_LIBRARY_DIRS}
        /Users/redwan/PycharmProjects/pypolo++/venv/lib
        )

add_subdirectory(ekf)
add_subdirectory(pf)
#add_subdirectory(lqr)