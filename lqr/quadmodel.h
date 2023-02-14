//
// Created by Redwan Newaz on 12/30/22.
//

#ifndef PARTICLEFILTER_QUADMODEL_H
#define PARTICLEFILTER_QUADMODEL_H
#include <Eigen/Dense>
/*
 * # The dynamics is from pp. 17, Eq. (2.22), https://www.kth.se/polopoly_fs/1.588039.1550155544!/Thesis%20KTH%20-%20Francesco%20Sabatino.pdf
# The constants is from Different Linearization Control Techniques for a Quadrotor System

# quadrotor physical constants
g = 9.81
m = 1.
Ix = 8.1 * 1e-3
Iy = 8.1 * 1e-3
Iz = 14.2 * 1e-3

def f(x, u):
    0   1   2   3    4   5   6     7      8       9      10    11
    x1, x2, y1, y2, z1, z2, phi1, phi2, theta1, theta2, psi1, psi2 = x.reshape(-1).tolist()
    ft, tau_x, tau_y, tau_z = u.reshape(-1).tolist()
    dot_x = np.array([
     x2,
     ft/m*(np.sin(phi1)*np.sin(psi1)+np.cos(phi1)*np.cos(psi1)*np.sin(theta1)),
     y2,
     ft/m*(np.cos(phi1)*np.sin(psi1)*np.sin(theta1)-np.cos(psi1)*np.sin(phi1)),
     z2,
     -g+ft/m*np.cos(phi1)*np.cos(theta1),
     phi2,
     (Iy-Iz)/Ix*theta2*psi2+tau_x/Ix,
     theta2,
     (Iz-Ix)/Iy*phi2*psi2+tau_y/Iy,
     psi2,
     (Ix-Iy)/Iz*phi2*theta2+tau_z/Iz])
    return dot_x
 */

enum class Axis{
    X = 0,
    Y,
    Z,
    Yaw
};

class QuadrotorModel
{
public:
    QuadrotorModel()
    {
        //[X-subsystem]: The state variables are x, dot_x, pitch, dot_pitch
        Ax.resize(4, 4);
        Ax <<   0.0, 1.0, 0.0, 0.0,
                0.0, 0.0,   g, 0.0,
                0.0, 0.0, 0.0, 1.0,
                0.0, 0.0, 0.0, 0.0;
        Bx.resize(4, 1);
        Bx << 0, 0, 0, 1.0 / Ix;


        //[Y-subsystem]: The state variables are y, dot_y, roll, dot_roll
        Ay.resize(4, 4);
        Ay <<   0.0, 1.0, 0.0, 0.0,
                0.0, 0.0,  -g, 0.0,
                0.0, 0.0, 0.0, 1.0,
                0.0, 0.0, 0.0, 0.0;
        By.resize(4, 1);
        By << 0, 0, 0, 1.0 / Iy;

        //[Z-subsystem]:  The state variables are z, dot_z
        Az.resize(2, 2);
        Az <<   0.0, 1.0,
                0.0, 0.0;
        Bz.resize(2, 1);
        Bz << 0, 1.0 / m;

        //[Yaw-subsystem]:  The state variables are yaw, dot_yaw
        Ayaw.resize(2, 2);
        Ayaw <<   0.0, 1.0,
                0.0, 0.0;
        Byaw.resize(2, 1);
        Byaw << 0, 1.0 / Iz;

    }

    Eigen::MatrixXd get_transition_mat(const Axis& axis)
    {
        switch (axis)
        {
            case Axis::X: return Ax;
            case Axis::Y: return Ay;
            case Axis::Z: return Az;
            case Axis::Yaw: return Ayaw;
        }
    }

    Eigen::MatrixXd get_control_mat(const Axis& axis)
    {
        switch (axis)
        {
            case Axis::X: return Bx;
            case Axis::Y: return By;
            case Axis::Z: return Bz;
            case Axis::Yaw: return Byaw;
        }
    }

    /**
     *
     * @param x 12D state vector <x1, x2, y1, y2, z1, z2, phi1, phi2, theta1, theta2, psi1, psi2>
     * @param u 4D control vector <ft, tau_x, tau_y, tau_z>
     * @return
     */
    Eigen::MatrixXd f(const Eigen::MatrixXd& x, const Eigen::VectorXd& uu)
    {
        Eigen::Vector4d ug{m * g, 0, 0, 0};
        Eigen::VectorXd u = uu + ug;
        Eigen::Matrix<double, 12, 1> x_dot;
        x_dot(0, 0) = x(1);
        x_dot(1, 0) = u(0) / m * (sin(x(6)) * sin(x(10)) + cos(x(6)) * cos(x(10) * sin(x(8))));
        x_dot(2, 0) = x(3);
        x_dot(3, 0) = u(0) / m * (cos(x(6)) * sin(x(10)) * sin(x(8)) - cos(x(10)) * sin(x(6)));
        x_dot(4, 0) = x(5);
        x_dot(5, 0) = -g + u(0) / m * cos(x(6)) * cos(x(8));
        x_dot(6, 0) = x(11);
        x_dot(7, 0) = (Iy - Iz) / Ix * x(9) * x(11) + u(1) / Ix;
        x_dot(8, 0) = x(9);
        x_dot(9, 0) =(Iz-Ix) / Iy * x(7) * x(11) + u(2) / Iy;
        x_dot(10, 0) = x(11);
        x_dot(11, 0) = (Ix-Iy) / Iz * x(7) * x(9) + u(3) / Iz;

        return x_dot;
    }

private:
    const double g = 9.81; // m/s
    const double m = 1.; // kg mass
    const double Ix = 8.1 * 1e-3; // inertia x axis
    const double Iy = 8.1 * 1e-3; // inertia y axis
    const double Iz = 14.2 * 1e-3; // inertia z axis

    Eigen::MatrixXd Ax, Ay, Az, Ayaw, Bx, By, Bz, Byaw;


};
#endif //PARTICLEFILTER_QUADMODEL_H
