#include <quadcopter_ompl_planner/quadcopter.hpp>

namespace quadcopter_ompl {

    void Quadcopter::rotation(const Eigen::Vector3d& angles) {
        const auto psi = angles(0);
        const auto theta = angles(1);
        const auto phi = angles(2);
        R << 
            cos(phi) * cos(theta), cos(phi) * sin(theta) * sin(psi) - cos(psi) * sin(phi), sin(phi) * sin(psi) + cos(phi) * cos(psi) * sin(theta),
            cos(theta) * sin(phi), cos(phi) * cos(psi) + sin(phi) * sin(theta) * sin(psi), cos(psi) * sin(phi) * sin(theta) - cos(phi) * sin(psi),
            - sin(theta), cos(theta) * sin(psi), cos(theta) * cos(psi);
    }

    void Quadcopter::thrust(const Eigen::Vector4d& inputs) {
        T << 0.0, 0.0, k * inputs.sum();
    }

    void Quadcopter::torques(const Eigen::Vector4d& inputs) {
        tau << L * k * (inputs(1) - inputs(3)),
               L * k * (inputs(2) - inputs(4)),
               b * (inputs(1) - inputs(2) + inputs(3) - inputs(4));
    }

    void Quadcopter::acceleration(const Eigen::Vector4d& inputs, const Eigen::Vector3d& angles) {
        rotation(angles);
        thrust(inputs);
        T *= R;
        Fd = -kd * xdot_;
        a = Eigen::RowVector3d(0,0,-g) + 1/m * T + Fd;
    }

    void Quadcopter::angular_acceleration(const Eigen::Vector4d& inputs) {
        torques(inputs);
        omegadot = I.inverse() * (tau - omega.cross(I * omega));
    }

    void Quadcopter::thetadot2omega(const Eigen::Vector3d& angles) {
        const auto phi = angles(0);
        const auto theta = angles(1);
        const auto psi = angles(2);
        Eigen::Matrix3d W;
        W << 1, 0, -sin(theta),
             0, cos(phi), cos(theta) * sin(phi),
             0, -sin(phi), cos(theta) * cos(phi);
        omega = W * thetadot;
    }

    void Quadcopter::omega2thetadot(const Eigen::Vector3d& angles) {
        const auto phi = angles(0);
        const auto theta = angles(1);
        const auto psi = angles(2);
        Eigen::Matrix3d W;
        W << 1, 0, -sin(theta),
             0, cos(phi), cos(theta) * sin(phi),
             0, -sin(phi), cos(theta) * cos(phi);
        thetadot = W.inverse() * omega;
    }

}

