//
// Created by redwan on 12/15/22.
//

#include "LQR.h"
#include <iostream>
#include <vector>
#include <Eigen/Core>
//https://github.com/sundw2014/Quadrotor_LQR/blob/master/3D_quadrotor.py
using namespace std;

namespace bebop2 {
    bool LQR::solveDARE(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B, const Eigen::MatrixXd &Q,
                        const Eigen::MatrixXd &R, Eigen::MatrixXd *X) const {
        // borrowed from https://github.com/wcaarls/grl/blob/master/addons/lqr/src/lqr.cpp

        Eigen::MatrixXd At = A.transpose(), Bt = B.transpose();
        *X = Q;

        double d = EPS;
        for (size_t ii=0; ii < MAX_ITERATION && d >= EPS; ++ii)
        {
            Eigen::MatrixXd Xp = *X;

            *X = Q + At*(*X)*A - At*(*X)*B*(Bt*(*X)*B+R).inverse()*Bt*(*X)*A;
            d = (*X - Xp).array().abs().sum();
        }

        return d <= EPS;
    }

    bool LQR::solve(const Eigen::MatrixXd& Xg, Eigen::MatrixXd& X0) {

        int index = 0;
        Eigen::MatrixXd X;
        A = quadModel_.get_transition_mat(Axis::X);
        B = quadModel_.get_control_mat(Axis::X);
        X.resize(A.rows(), 1);
        X(0) = Xg(index);
//        [signalx[idx], 0, 0, 0]) - x[[0, 1, 8, 9]]

        Q.resize(A.rows(), A.rows());
        Q(0, 0) = 10. ;
        if (!solveDARE(A, B, Q, R, &X))
        {
            WARNING("Could not solve DARE: error ");
            return false;
        }
        DEBUG(X);
        // Compute feedback gain matrix
        Eigen::MatrixXd K = (B.transpose()* X * B + R).inverse()*(B.transpose() * X * A);

//        Eigen::MatrixXd U = -K * ERROR;
        return true;
    }

    void LQR::compute_control(const Eigen::MatrixXd& Xg, Eigen::MatrixXd& X0)
    {
//        UX = Ks[0].dot(np.array([signalx[idx], 0, 0, 0]) - x[[0, 1, 8, 9]])[0]
//        UY = Ks[1].dot(np.array([signaly[idx], 0, 0, 0]) - x[[2, 3, 6, 7]])[0]
//        UZ = Ks[2].dot(np.array([signalz[idx], 0]) - x[[4, 5]])[0]
//        UYaw = Ks[3].dot(np.array([signalyaw[idx], 0]) - x[[10, 11]])[0]

// # closed-loop dynamics. u should be a function
//    x = np.array(x)
//    X, Y, Z, Yaw = x[[0, 1, 8, 9]], x[[2, 3, 6, 7]], x[[4, 5]], x[[10, 11]]
//    UZ, UY, UX, UYaw = u(x, t).reshape(-1).tolist()
//    dot_X = Ax.dot(X) + (Bx * UX).reshape(-1)
//    dot_Y = Ay.dot(Y) + (By * UY).reshape(-1)
//    dot_Z = Az.dot(Z) + (Bz * UZ).reshape(-1)
//    dot_Yaw = Ayaw.dot(Yaw) + (Byaw * UYaw).reshape(-1)
//    dot_x = np.concatenate(
//        [dot_X[[0, 1]], dot_Y[[0, 1]], dot_Z, dot_Y[[2, 3]], dot_X[[2, 3]], dot_Yaw])
//    return dot_x



        Eigen::VectorXd Ux,Xx, Uy, Xy;
        Eigen::VectorXd Uz, Xz, Uyaw, Xyaw;

        std::vector<int> Xind{0, 1, 8, 9}, Yind{2, 3, 6, 7}, Zind{4, 5}, Yawind{10, 11};

        auto populate = [&X0](const vector<int>& indexes, Eigen::VectorXd& Xx){
            Xx.resize(indexes.size(), 1);
            int j = 0;
            for(auto i: indexes)
                Xx(j++) = X0(i, 0);
        };

        populate(Xind, Xx);
        populate(Yind, Xy);
        populate(Zind, Xz);
        populate(Yawind, Xyaw);

        Ux.resize(Xx.rows());
        Uy.resize(Xy.rows());
        Uz.resize(Xz.rows());
        Uyaw.resize(Xyaw.rows());

        Ux.setZero();
        Uy.setZero();
        Uz.setZero();
        Uyaw.setZero();


        Ux(0) = Xg(0);
        Uy(1) = Xg(1);
        Uz(0) = Xg(2);
        Uyaw(0) = Xg(3);

        Ux = Ux - Xx;
        Uy = Uy - Xy;
        Uz = Uz - Xz;
        Uyaw = Uyaw - Xyaw;

        std::vector<Eigen::VectorXd>Error{Ux, Uy, Uz, Uyaw};

        std::vector<Eigen::VectorXd>State{Xx, Xy, Xz, Xyaw};

//        std::vector<Eigen::MatrixXd>Ks;
        Eigen::Vector4d UU;
//        UZ, UY, UX, UYaw]
        std::vector<int> indexMap{2, 1, 0, 3};

        for (int j = 0; j < State.size(); ++j) {
            auto axis = static_cast<Axis>(j);
            A = quadModel_.get_transition_mat(axis);
            B = quadModel_.get_control_mat(axis);

            Eigen::MatrixXd X = State[j];
            Q.resize(A.rows(), A.rows());
            Q(0, 0) = 0.1 ;
            if (!solveDARE(A, B, Q, R, &X))
            {
                WARNING("Could not solve DARE: error ");
                return;

            }

            Eigen::MatrixXd K = (B.transpose()* X * B + R).inverse()*(B.transpose() * X * A);

//            K = np.matrix(scipy.linalg.inv(R) * (B.T * X))
//            Eigen::MatrixXd K = (R.inverse() * (B.transpose() * X));
            Eigen::MatrixXd res = -K * Error.at(j);
            UU(indexMap[j]) =  res(0);

        }

        X0 = X0 + quadModel_.f(X0, UU ) * dt_;

    }

    LQR::LQR() {
        dt_ = 0.03;
        R.resize(1, 1);
        R.setIdentity();
    }


} // bebop2