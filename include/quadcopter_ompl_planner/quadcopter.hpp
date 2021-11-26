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
        void acceleration();
        void angular_acceleration();

    private:
        double L, m, k, g;
        Eigen::Vector3d x;
        Eigen::Vector3d xdot;
        Eigen::Vector3d theta;
        Eigen::Vector3d thetadot;
        Eigen::Matrix3d omega;
        Eigen::Matrix3d R;

};
}
