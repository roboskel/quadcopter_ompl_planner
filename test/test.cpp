#include "quadcopter_ompl_planner/quadcopter_ompl.hpp"

int main() {
    quadcopter_ompl::QuadcopterOMPL q;
    q.setStart(0,0,0,0,0,0,1);
    q.setGoal(1,1,1,0,0,0,1);
    auto solution = q.solve(1);
    if (std::get<0>(solution)) {
        std::get<1>(solution).asGeometric().printAsMatrix(std::cout);
    }
    else {
        std::cout << "Could not find a solution" << std::endl;
    }
    return 0;
}
