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

        setup = std::make_shared<ompl::control::SimpleSetup>(control_space);
        setup->setStateValidityChecker(
            [&](const ompl::base::State* s) {
                return this->isStateValid(s);
            }
        );

        solver = std::make_shared<ompl::control::ODEBasicSolver<>>(setup->getSpaceInformation(), 
            [&](const ompl::control::ODESolver::StateType& s, const ompl::control::Control* c, ompl::control::ODESolver::StateType& r) {
                this->quadKinematicModelSolver(s,c,r);
            }
        );
        setup->setStatePropagator(ompl::control::ODESolver::getStatePropagator(solver,
            [&](const ompl::base::State* s, const ompl::control::Control* c, const double d, ompl::base::State* r) {
                this->postIntegrationCallback(s,c,d,r);
            }
        ));
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

    std::shared_ptr<ompl::control::SimpleSetup> QuadcopterOMPL::getSetup() {
        return this->setup;
    }

    void QuadcopterOMPL::setStart(double x, double y, double z, double qx, double qy, double qz, double qw) {
        ompl::base::ScopedState<ompl::base::SE3StateSpace> s(state_space);
        s->setXYZ(x,y,z);
        s->rotation().x = qx;
        s->rotation().y = qy;
        s->rotation().z = qz;
        s->rotation().w = qw;
        setup->setStartState(s);
    }

    void QuadcopterOMPL::setGoal(double x, double y, double z, double qx, double qy, double qz, double qw) {
        ompl::base::ScopedState<ompl::base::SE3StateSpace> s(state_space);
        s->setXYZ(x,y,z);
        s->rotation().x = qx;
        s->rotation().y = qy;
        s->rotation().z = qz;
        s->rotation().w = qw;
        setup->setGoalState(s);
    }

    void QuadcopterOMPL::setStartAndGoal(double sx, double sy, double sz, double sqx, double sqy, double sqz, double sqw, double gx, double gy, double gz, double gqx, double gqy, double gqz, double gqw) {
        ompl::base::ScopedState<ompl::base::SE3StateSpace> ss(state_space);
        ss->setXYZ(sx,sy,sz);
        ss->rotation().x = sqx;
        ss->rotation().y = sqy;
        ss->rotation().z = sqz;
        ss->rotation().w = sqw;

        ompl::base::ScopedState<ompl::base::SE3StateSpace> sg(state_space);
        sg->setXYZ(gx,gy,gz);
        sg->rotation().x = gqx;
        sg->rotation().y = gqy;
        sg->rotation().z = gqz;
        sg->rotation().w = gqw;

        setup->setStartAndGoalStates(ss,sg, this->resolution);
    }

    std::tuple<ompl::base::PlannerStatus, ompl::control::PathControl> QuadcopterOMPL::solve(double duration) {
        ompl::base::PlannerStatus ps = setup->solve(duration);
        ompl::control::PathControl pc(setup->getSpaceInformation());
        if (setup->haveSolutionPath()) {
            pc = setup->getSolutionPath();
        }
        return std::tuple<ompl::base::PlannerStatus, ompl::control::PathControl>(ps,pc);
    }
}
