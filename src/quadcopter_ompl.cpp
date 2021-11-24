#include "quadcopter_ompl_planner/quadcopter_ompl.hpp"

namespace quadcopter_ompl {

    QuadcopterOMPL::QuadcopterOMPL() {
        init();
    }

    QuadcopterOMPL::~QuadcopterOMPL() {}

    void QuadcopterOMPL::init() {
        this->resolution = 0.01;
        // State space
        state_space = std::make_shared<ompl::base::SE3StateSpace>();
        ompl::base::RealVectorBounds state_bounds(3);
        // TODO investigate this
        // the rationale is that we search "infinitely"
        // (maybe bounds can also be set based on the start-goal bounding box)
        state_bounds.setLow(-999);
        state_bounds.setHigh(999);
        state_space->setBounds(state_bounds);

        // Control space
        control_space = std::make_shared<ompl::control::RealVectorControlSpace>(state_space, 4);
        ompl::base::RealVectorBounds control_bounds(4);
        control_bounds.setLow(-1.0);
        control_bounds.setHigh(1.0);
        control_space->setBounds(control_bounds);

        // NOTE I could initialize a SimpleSetup here and get the SpaceInformation object
        space_information = std::make_shared<ompl::control::SpaceInformation>(state_space, control_space);
        space_information->setStateValidityChecker(
            [&](const ompl::base::State* s) {
                return this->isStateValid(s);
            }
        );

        solver = std::make_shared<ompl::control::ODEBasicSolver<>>(space_information, 
            [&](const ompl::control::ODESolver::StateType& s, const ompl::control::Control* c, ompl::control::ODESolver::StateType& r) {
                this->quadKinematicModelSolver(s,c,r);
            }
        );
        space_information->setStatePropagator(ompl::control::ODESolver::getStatePropagator(solver,
            [&](const ompl::base::State* s, const ompl::control::Control* c, const double d, ompl::base::State* r) {
                this->postIntegrationCallback(s,c,d,r);
            }
        ));

        space_information->setup();

    }

    void QuadcopterOMPL::quadKinematicModelSolver(const ompl::control::ODESolver::StateType& state, const ompl::control::Control* control, ompl::control::ODESolver::StateType& res) {
        // TODO implement the kinematic model here
    }
    
    void QuadcopterOMPL::postIntegrationCallback(const ompl::base::State* state, const ompl::control::Control* control, const double duration, ompl::base::State* res) {
        // TODO
    }

    bool QuadcopterOMPL::isStateValid(const ompl::base::State* state) {
        // TODO
        return true;
    }

    void QuadcopterOMPL::setResolution(double r) {
        this->resolution = r;
    }
    
    double QuadcopterOMPL::getResolution() {
        return this->resolution;
    }

}
