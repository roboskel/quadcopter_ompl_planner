#pragma once

#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/control/ODESolver.h>
#include <ompl/config.h>
#include <iostream>
#include <valarray>
#include <limits>

namespace quadcopter_ompl {

class QuadcopterOMPL {

    public:
        QuadcopterOMPL();
        ~QuadcopterOMPL();

};

}
