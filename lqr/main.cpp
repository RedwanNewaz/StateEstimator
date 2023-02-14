//
// Created by redwan on 12/15/22.
//
#include <iostream>
#include "LQR.h"

using namespace std;
using namespace bebop2;

int main(int argc, char* argv[])
{
    LQR lqr;
    Eigen::MatrixXd X0, Xg;
    X0.resize(12, 1);
    Xg.resize(12, 1);
    X0.setZero();
    Xg.setZero();
    Xg(0) = Xg(1) = Xg(2) = 1.0;
    Eigen::MatrixXd error = Xg - X0;
    int step = 25;
    do{
        lqr.compute_control(Xg, X0);
        cout << "[LQR Control] " << step << " " <<  X0.transpose() << endl;
    }while (--step >=0);

    return 0;
}
