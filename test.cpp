#include <ompl/control/SimpleSetup.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/tools/benchmark/Benchmark.h>
#include <omplapp/apps/KinematicCarPlanning.h>
#include <omplapp/apps/DynamicCarPlanning.h>
#include <omplapp/config.h>
#include <boost/math/constants/constants.hpp>
// #include <boost/units/quantity.hpp>
// #include <boost/units/systems/si/plane_angle.hpp>
// #include <boost/units/systems/angle/degrees.hpp>
// #include <boost/units/base_units/angle/degree.hpp>

#include <stdlib.h>
#include <fstream>
#include <iostream>

// #include "CarStatePropagator.cpp"

namespace ob = ompl::base;
namespace oc = ompl::control;
namespace og = ompl::geometric;


class DemoControlSpace : public oc::RealVectorControlSpace
{
    public:
        DemoControlSpace(const ob::StateSpacePtr &stateSpace) : oc::RealVectorControlSpace(stateSpace, 2)
        {
        }
};

double deg2Rad(double angleInDegrees)
{
    return angleInDegrees * boost::math::constants::pi<double>() / 180;
}

void kinematicCarSetup(ompl::app::KinematicCarPlanning &setup)
{
    ob::StateSpacePtr SE2(setup.getStateSpace());

    ob::RealVectorBounds bounds(2);
    bounds.setLow(-10);
    bounds.setHigh(10);
    SE2->as<ob::SE2StateSpace>()->setBounds(bounds);

    ob::ScopedState<ob::SE2StateSpace> start(SE2);
    start->setX(0);
    start->setY(0);
    start->setYaw(0);

    ob::ScopedState<ob::SE2StateSpace> goal(SE2);
    goal->setX(20);
    goal->setY(20);
    goal->setYaw(boost::math::constants::pi<double>() * 10);

    setup.setStartAndGoalStates(start, goal, .1);

    // Create state propagator
    // CarStatePropagator propagator; 
    // setup.setStatePropagator(0);


    setup.setPlanner(std::make_shared<oc::KPIECE1>(setup.getSpaceInformation()));
    std::vector<double> cs(2);
    cs[0] = cs[1] = 0.1;
    setup.setup();
    setup.getStateSpace()->getDefaultProjection()->setCellSizes(cs);
}

void kinematicCarDemo(ompl::app::KinematicCarPlanning &setup)
{
    // Try to solve
    if (setup.solve(20))
    {
        // print the solution path: prints states along the paths and controls required to get from
        // one state to the next. 
        oc::PathControl &path(setup.getSolutionPath());
        // path.interpolate(); // make the path plotable 
        path.printAsMatrix(std::cout);

        // Printing as geometric path to file
        const ompl::geometric::PathGeometric &geo_path = path.asGeometric();
        std::cout << "AS Geomtric path:" << std::endl;
        std::ofstream file ("geometric.path");
        if(file.is_open())
        {
            geo_path.printAsMatrix( file);
            file.close();
        }
        else std::cout << "Unable to open file" << std::endl;
        // geo_path.printAsMatrix(std::cout);

        if (!setup.haveExactSolutionPath())
        {
            std::cout << "Solution is approximate. Distance to actual goal is " 
                      << setup.getProblemDefinition()->getSolutionDifference() << std::endl;
        }
    }
}

void kinematicCarBenchmark(ompl::app::KinematicCarPlanning &setup)
{
    ompl::tools::Benchmark::Request request(20.0, 1000.0, 10.0); // runtime (s), memory (MB), run count

    ompl::tools::Benchmark b(setup, setup.getName());
    b.addPlanner(std::make_shared<oc::RRT>(setup.getSpaceInformation()));
    b.addPlanner(std::make_shared<oc::KPIECE1>(setup.getSpaceInformation()));
    b.benchmark(request);
    b.saveResultsToFile();
}

void KinematicCarODE(const oc::ODESolver::StateType& q, const oc::Control* control, oc::ODESolver::StateType& qdot)
{
    const double *u = control->as<oc::RealVectorControlSpace::ControlType>()->values;
    const double theta = q[2];
    double carLength = 1;
    // Zero out qdot
    qdot.resize (q.size(), 0);

    // reduce steering between [-30;-15]
    double steer = u[1];
    double rad_max = deg2Rad(-15);
    double rad_min = deg2Rad(-30);
    if (steer > rad_max) steer = rad_max;
    else if (steer < rad_min) steer = rad_min;

    qdot[0] = 2 * u[0] * cos(theta);
    qdot[1] = 2 * u[0] * sin(theta);
    qdot[2] = 2 * u[0] * tan(steer) / carLength;
}

void KinematicCarPostIntegration(const ob::State* /*state*/, const oc::Control* /*control*/, const double /*duration*/, ob::State *result)
{
    // Normalized orientation between 0 and 2 pi
    ob::SO2StateSpace SO2;
    SO2.enforceBounds(result->as<ob::SE2StateSpace::StateType>()->as<ob::SO2StateSpace::StateType>(1));
}

bool isStateValid(const oc::SpaceInformation *si, const ob::State *state)
{
    // base::ScopedState<base::SE2StateSpace>
    const auto *se2state = state->as<ob::SE2StateSpace::StateType>();

    const auto *pos = se2state->as<ob::RealVectorStateSpace::StateType>(0);

    const auto *rot = se2state->as<ob::SO2StateSpace::StateType>(1);

    return true;
}

void planWithSimpleSetup(oc::SimpleSetup &ss)
{
    // set state validity checking for this space
    oc::SpaceInformation *si = ss.getSpaceInformation().get();
    ss.setStateValidityChecker(
        [si](const ob::State *state) { return isStateValid(si, state); });
    
    // Settin propagation routine for this space: does not use ODESolver
    // ss.setStatePropagator(std::make_shared<CarStatePropagator>(ss.getSpaceInformation()));

    // Using ODESolver to propagate the system. Calls KinematicCarPostIntegration to normalize the orientation values after integration has finished.
    auto odeSolver(std::make_shared<oc::ODEBasicSolver<>>(ss.getSpaceInformation(), &KinematicCarODE));
    ss.setStatePropagator(oc::ODESolver::getStatePropagator(odeSolver, &KinematicCarPostIntegration));

    ss.setPlanner(std::make_shared<oc::RRT>(ss.getSpaceInformation()));

    ss.setup();

    ob::PlannerStatus solved = ss.solve(10.0);
    if(solved)
    {
        std::cout << "Found solution:" << std::endl;
        ss.getSolutionPath().asGeometric().printAsMatrix(std::cout);
        // Printing as geometric path to file
        const ompl::geometric::PathGeometric &geo_path = ss.getSolutionPath().asGeometric();
        std::cout << "Writing solution to 'geometric.path':" << std::endl;
        std::ofstream file ("geometric.path");
        if(file.is_open())
        {
            geo_path.printAsMatrix( file);
            file.close();
        }
        else std::cout << "Unable to open file" << std::endl;
    }
    else
        std::cout << "No solution found" << std::endl;
}

int main(int argc, char** argv)
{
    if (argc < 6) 
    {
        std::cout << "Usage: path_planner startX startY startYaw goalX goalY goalYaw" << std::endl;
        return 0;
    }

    int i = 0;
    double startX = atof(argv[++i]);
    double startY = atof(argv[++i]);
    double startYaw = atof(argv[++i]);
    double goalX = atof(argv[++i]);
    double goalY = atof(argv[++i]);
    double goalYaw = atof(argv[++i]);
    std::cout << startX << " " << goalYaw << std::endl;

    // Create SimpleSetup
    auto space(std::make_shared<ob::SE2StateSpace>());

    ob::RealVectorBounds bounds(2);
    bounds.setLow(-25);
    bounds.setHigh(25);

    space->setBounds(bounds);

    // create control space
    // auto cspace(std::make_shared<control::RealVectorControlSpace>(space, 2));
    auto cspace(std::make_shared<DemoControlSpace>(space));
    // set bounds
    ob::RealVectorBounds cbounds(2);
    cbounds.setLow(-1.0); // limits steering angle AND speed
    cbounds.setHigh(1.0);

    cspace->setBounds(cbounds);

    // define simple setup class
    oc::SimpleSetup ss(cspace);

    // Set start and goal
    ob::ScopedState<ob::SE2StateSpace> start(space);
    start->setX(startX);
    start->setY(startY);
    start->setYaw(startYaw);

    ob::ScopedState<ob::SE2StateSpace> goal(space);
    goal->setX(goalX);
    goal->setY(goalY);
    goal->setYaw(goalYaw);

    ss.setStartAndGoalStates(start, goal, 0.05);

    planWithSimpleSetup(ss);
    return 0;
}

// int main(int argc, char** /* argv */)
// {
//     app::KinematicCarPlanning regularCar;
//     // control::ControlSpace space; 

//     kinematicCarSetup(regularCar);

//     // If any command line args are given, solve the problem multiple times with different planners
//     // and collect benchmark stats. Otherwise, solve the problem once for each car type and print the path.
//     if (argc > 1)
//         kinematicCarBenchmark(regularCar);
//     else
//         kinematicCarDemo(regularCar);
    
//     return 0;
// }

// int main()
// {
//     // plan in SE2
//     app::SE2RigidBodyPlanning setup;

//     // Load robot
//     std::string robot_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/2D/car1_planar_robot.dae";
//     std::string env_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/2D/Maze_planar_env.dae";
//     setup.setRobotMesh(robot_fname);
//     setup.setEnvironmentMesh(env_fname);

//     // define starting state
//     base::ScopedState<base::SE2StateSpace> start(setup.getSpaceInformation());
//     start->setX(0.0);
//     start->setY(0.0);

//     // define goal
//     base::ScopedState<base::SE2StateSpace> goal(start);
//     goal->setX(26.0);
//     goal->setY(0.0);

//     // set start & goal state
//     setup.setStartAndGoalStates(start, goal);

//     // attempt to solve the problem and print it to screen if a solution is found
//     if(setup.solve())
//         setup.getSolutionPath().print(std::cout);

//     return 0;
// }