#pragma once

#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/control/ODESolver.h>
#include <ompl/config.h>
#include <iostream>
#include <limits>

namespace quadcopter_ompl {

class QuadcopterOMPL {

    public:
        QuadcopterOMPL();
        ~QuadcopterOMPL();

    protected:
        void quadKinematicModelSolver(const ompl::control::ODESolver::StateType&, const ompl::control::Control*, ompl::control::ODESolver::StateType&);
        void postIntegrationCallback(const ompl::base::State*, const ompl::control::Control*, const double, ompl::base::State*);
        bool isStateValid(const ompl::base::State* state);
        void setResolution(double);
        double getResolution();
        void init();

    private:
        double resolution;
        std::shared_ptr<ompl::control::ODEBasicSolver<>> solver;
        std::shared_ptr<ompl::base::SE3StateSpace> state_space;
        std::shared_ptr<ompl::control::RealVectorControlSpace> control_space;
        std::shared_ptr<ompl::control::SpaceInformation> space_information;
};

}
