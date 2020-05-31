#include <ompl/control/SimpleSetup.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/tools/benchmark/Benchmark.h>
#include <omplapp/apps/KinematicCarPlanning.h>
#include <omplapp/apps/DynamicCarPlanning.h>
#include <ompl/base/spaces/SE2StateSpace.h>
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
// Max and min steer of car are fixed
#define ROBOT_LENGTH 1.0
#define MAX_STEER -15.0
#define MIN_STEER -30.0
#define STATESPACE_BOUND_HIGH 50.0
#define STATESPACE_BOUND_LOW -STATESPACE_BOUND_HIGH
#define CSPACE_BOUND_HIGH 1.0
#define CSPACE_BOUND_LOW -CSPACE_BOUND_HIGH

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

void KinematicCarODE(const oc::ODESolver::StateType& q, const oc::Control* control, oc::ODESolver::StateType& qdot);
void KinematicCarPostIntegration(const ob::State* /*state*/, const oc::Control* /*control*/, const double /*duration*/, ob::State *result);
bool isStateValid(const oc::SpaceInformation *si, const ob::State *state);

double deg2Rad(double angleInDegrees)
{
    return angleInDegrees * boost::math::constants::pi<double>() / 180;
}

void kinematicCarSetup(ompl::app::KinematicCarPlanning &setup)
{
    ob::StateSpacePtr SE2(setup.getStateSpace());

    // ob::RealVectorBounds bounds(2);
    // bounds.setLow(-50);
    // bounds.setHigh(50);
    // SE2->as<ob::SE2StateSpace>()->setBounds(bounds);

    oc::SpaceInformation *si = setup.getSpaceInformation().get();
    setup.setStateValidityChecker(
        [si](const ob::State *state) { return true; });

    // Using ODESolver to propagate the system. Calls KinematicCarPostIntegration to normalize the orientation values after integration has finished.
    auto odeSolver(std::make_shared<oc::ODEBasicSolver<>>(setup.getSpaceInformation(), &KinematicCarODE));
    setup.setStatePropagator(oc::ODESolver::getStatePropagator(odeSolver, &KinematicCarPostIntegration));

    ob::ScopedState<ob::SE2StateSpace> start(SE2);
    start->setX(50);
    start->setY(-50);
    start->setYaw(0.3);

    ob::ScopedState<ob::SE2StateSpace> goal(SE2);
    goal->setX(130);
    goal->setY(-100);
    goal->setYaw(0.61);

    setup.setStartAndGoalStates(start, goal, 0.1);

    std::string env_fname = "./ompl_install/resources/2D/Barriers_easy_env.dae";
    std::string robot_fname = "./ompl_install/resources/2D/car1_planar_robot.dae";
    setup.setEnvironmentMesh(env_fname);
    setup.setRobotMesh(robot_fname);

    // auto ccheck = setup.getGeometrySpecification();

    setup.setPlanner(std::make_shared<oc::KPIECE1>(setup.getSpaceInformation()));
    std::vector<double> cs(2);
    cs[0] = cs[1] = 0.1;
    setup.setup();
    setup.getStateSpace()->getDefaultProjection()->setCellSizes(cs);
}

void kinematicCarDemo(ompl::app::KinematicCarPlanning &setup)
{
    // Try to solve
    if (setup.solve(100))
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
    // Zero out qdot
    qdot.resize (q.size(), 0);

    // reduce steering between [-30;-15]
    double steer = u[1];
    double rad_max = deg2Rad(MAX_STEER);
    double rad_min = deg2Rad(MIN_STEER);
    if (steer > rad_max) steer = rad_max;
    else if (steer < rad_min) steer = rad_min;

    qdot[0] = u[0] * cos(theta);
    qdot[1] = u[0] * sin(theta);
    qdot[2] = u[0] * tan(steer) / ROBOT_LENGTH;
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

    if (pos->values[0] < 10 && pos->values[1] > 1 && pos->values[1] < 10)
        return false;

    return true;
}

void planWithSimpleSetup(oc::SimpleSetup &ss, double time_limit, int planner_idx)
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

    if (planner_idx == 0)
        ss.setPlanner(std::make_shared<oc::KPIECE1>(ss.getSpaceInformation()));
    else
        ss.setPlanner(std::make_shared<oc::RRT>(ss.getSpaceInformation()));

    ss.setup();

    ob::PlannerStatus solved;
    if (time_limit > 0.0)
        solved = ss.solve(time_limit);
    else
    {
        ss.solve(ob::exactSolnPlannerTerminationCondition (ss.getProblemDefinition()));
    }
    
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

int ompl_planer(int argc, char** argv)
{
    if (argc < 7) 
    {
        std::cout << "Usage: path_planner startX startY startYaw goalX goalY goalYaw [-time limit | -exact] [-planner id]" << std::endl;
        std::cout << "-planner id: 0 KPIECE1, 1: RRT" << std::endl;
        std::cout << "-exact: run planner until an exact solution is found." << std::endl;
        return 0;
    }

    int i = 0;
    double startX = atof(argv[++i]);
    double startY = atof(argv[++i]);
    double startYaw = deg2Rad( atof(argv[++i]) );
    double goalX = atof(argv[++i]);
    double goalY = atof(argv[++i]);
    double goalYaw = deg2Rad( atof(argv[++i]) );

    double time_limit = 50.0;
    int planner_idx = 0;
    while (argc > ++i)
    {
        char *arg = argv[i];
        // std::cout << arg << std::endl;
        if (strcmp(arg, "-time") == 0)
            time_limit = atof(argv[++i]);
        else if (strcmp(arg, "-exact") == 0)
            time_limit = -1.0;
        else if (strcmp(arg, "-planner") == 0)
            planner_idx = atof(argv[++i]);
    }

    if (startX > STATESPACE_BOUND_HIGH or startY > STATESPACE_BOUND_HIGH or startX < STATESPACE_BOUND_LOW or startY < STATESPACE_BOUND_LOW
        or goalX > STATESPACE_BOUND_HIGH or goalY > STATESPACE_BOUND_HIGH or goalX < STATESPACE_BOUND_LOW or goalY < STATESPACE_BOUND_LOW)
        {
            std::cout << "startX, startY, goalX, goalY should belong to the interval [" << STATESPACE_BOUND_LOW << "; " << STATESPACE_BOUND_HIGH << "]" << std::endl;
            return 1;
        }

    // std::cout << startX << " " << goalX << std::endl;
    // Create SimpleSetup
    auto space(std::make_shared<ob::SE2StateSpace>());
    

    ob::RealVectorBounds bounds(2);
    bounds.setLow(STATESPACE_BOUND_LOW);
    bounds.setHigh(STATESPACE_BOUND_HIGH);

    space->setBounds(bounds);

    // create control space
    // auto cspace(std::make_shared<control::RealVectorControlSpace>(space, 2));
    auto cspace(std::make_shared<DemoControlSpace>(space));
    // set bounds
    ob::RealVectorBounds cbounds(2);
    cbounds.setLow(CSPACE_BOUND_LOW); // limits steering angle AND speed
    cbounds.setHigh(CSPACE_BOUND_HIGH);

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

    ss.setStartAndGoalStates(start, goal, 0.1);

    planWithSimpleSetup(ss, time_limit, planner_idx);
    return 0;
}

int omplapp_planner(int argc, char** argv)
{
    // Create cspace
    auto space(std::make_shared<ob::SE2StateSpace>());
    
    ob::RealVectorBounds bounds(2);
    bounds.setLow(STATESPACE_BOUND_LOW);
    bounds.setHigh(STATESPACE_BOUND_HIGH);

    space->setBounds(bounds);

    // create control space
    // auto cspace(std::make_shared<oc::RealVectorControlSpace>(space, 2));
    auto cspace(std::make_shared<DemoControlSpace>(space));
    ompl::app::KinematicCarPlanning regularCar(cspace);
    // control::ControlSpace space; 

    kinematicCarSetup(regularCar);

    // If any command line args are given, solve the problem multiple times with different planners
    // and collect benchmark stats. Otherwise, solve the problem once for each car type and print the path.
    // if (argc > 1)
    //     kinematicCarBenchmark(regularCar);
    // else
    kinematicCarDemo(regularCar);
    
    return 0;
}

int main(int argc, char** argv)
{
    if (argc > 1)
        return ompl_planer(argc, argv);

    std::cout << "Starting OMPL.app implementation: \n" << std::endl;
    return omplapp_planner(argc, argv);
}