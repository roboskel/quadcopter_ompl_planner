#pragma once

#include <Eigen/Dense>

namespace quadcopter_ompl {
class Quadcopter {

    public:
        Quadcopter();
        //Quadcopter(/*TODO*/);
        ~Quadcopter();
        void step(/*TODO*/);

    protected:
        void thetadot2omega(const Eigen::Vector3d&);
        void omega2thetadot(const Eigen::Vector3d&);
        void angular_acceleration(const Eigen::Vector4d&);
        void rotation(const Eigen::Vector3d&);
        void thrust(const Eigen::Vector4d&);
        void torques(const Eigen::Vector4d&);
        void acceleration(const Eigen::Vector4d&, const Eigen::Vector3d&);

    private:
        // Quadcopter properties
        // arm length, mass, thrust coefficient
        double L, m, k;
        // NOTE physics (might need to remove them or apply them to ODE)
        // gravity, drag coefficient, global drag coefficient
        double g, b, kd;
        Eigen::Vector3d a;
        Eigen::Vector3d x;
        Eigen::RowVector3d T;
        Eigen::RowVector3d Fd;
        Eigen::Vector3d tau;
        Eigen::Vector3d xdot;
        Eigen::Vector3d xdot_;
        Eigen::Vector3d theta;
        Eigen::Vector3d thetadot;
        Eigen::Vector3d omega;
        Eigen::Vector3d omegadot;
        Eigen::Matrix3d R;
        Eigen::DiagonalMatrix<double,3,3> I;
        // Eigen::Matrix3d I;

};
}
