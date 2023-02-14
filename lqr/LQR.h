//
// Created by redwan on 12/15/22.
//

#ifndef BEBOP2_CONTROLLER_LQR_H
#define BEBOP2_CONTROLLER_LQR_H
#include <iostream>
#include <Eigen/Dense>
#include "quadmodel.h"

#define EPS 0.001
#define MAX_ITERATION 1000
#define WARNING(x) std::cerr << x << std::endl
#define DEBUG(x) std::cout << x << std::endl

namespace bebop2 {

    class LQR {
    public:
        LQR();
        bool solve(const Eigen::MatrixXd& Xg, Eigen::MatrixXd& X0);
        void compute_control(const Eigen::MatrixXd& Xg, Eigen::MatrixXd& X0);

    private:

        bool solveDARE(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B, const Eigen::MatrixXd &Q, const Eigen::MatrixXd &R, Eigen::MatrixXd *X) const;
        Eigen::MatrixXd A, B, Q, R;
        double dt_;

        QuadrotorModel quadModel_;

    };

} // bebop2

#endif //BEBOP2_CONTROLLER_LQR_H
