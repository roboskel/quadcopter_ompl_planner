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
        void thrust();
        void torque();
        void rotation();
        void acceleration();
        void thetadot2omega();
        void omega2thetadot();
        void angular_acceleration();

    private:
        // Quadcopter properties
        // arm length, mass, thrust coefficient
        double L, m, k;
        // NOTE physics (might need to remove them or aplpy them to ODE)
        // gravity, drag coefficient
        double g, kd;
        Eigen::Vector3d x;
        Eigen::Vector3d xdot;
        Eigen::Vector3d theta;
        Eigen::Vector3d thetadot;
        Eigen::Matrix3d omega;
        Eigen::Matrix3d R;

};
}
