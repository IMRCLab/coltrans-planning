
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>

#include <ompl/config.h>

#include <yaml-cpp/yaml.h>
#include <boost/program_options.hpp>
#include <iostream>
#include <fstream>

namespace ob = ompl::base;
namespace og = ompl::geometric;

bool isStateValid(const ob::State *state)
{
    // cast the abstract state type to the type we expect
    const auto *se3state = state->as<ob::SE3StateSpace::StateType>();

    // extract the first component of the state and cast it to what we expect
    const auto *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

    // extract the second component of the state and cast it to what we expect
    const auto *rot = se3state->as<ob::SO3StateSpace::StateType>(1);

    // check validity of state defined by pos & rot


    // return a value that is always true but uses the two variables we define, so we avoid compiler warnings
    return (const void*)rot != (const void*)pos;
}

ob::OptimizationObjectivePtr getPathLengthObjective(const ob::SpaceInformationPtr& si)
{
    return ob::OptimizationObjectivePtr(new ob::PathLengthOptimizationObjective(si));
}

void payloadPlan(const std::vector<double>& start, const std::vector<double>& goal,
                const std::vector<double>& envmin, const std::vector<double>& envmax,
                const size_t &plannerType, const float &timelimit, std::string outputFile)
{
    // construct the state space
    auto space(std::make_shared<ob::SE3StateSpace>());
    // set the bounds for the R^3 part of SE(3)

    ob::RealVectorBounds bounds(3);
    // prepare the bounds
    for (size_t i=0; i < envmin.size(); ++i) {
        bounds.setLow(i, envmin[i]);
        bounds.setHigh(i,envmax[i]);
    } 
    // set the bounds
    space->setBounds(bounds);
    
    // construct an instance of space information from this state
    auto si(std::make_shared<ob::SpaceInformation>(space));
    // setup the state space information
    si->setup();
    // set state validity checking for this space
    si->setStateValidityChecker(isStateValid);
    
    // create a problem instance
    auto pdef(std::make_shared<ob::ProblemDefinition>(si));

    // create and set a start state
    auto startState = si->allocState();
    si->getStateSpace()->copyFromReals(startState, start);
    si->enforceBounds(startState);
    pdef->addStartState(startState);
    si->freeState(startState);
    
    // create and set a goal state
    auto goalState = si->allocState();
    si->getStateSpace()->copyFromReals(goalState, goal);
    si->enforceBounds(goalState);
    pdef->setGoalState(goalState);
    si->freeState(goalState);

    std::shared_ptr<ob::Planner> planner;
    //create planner
    switch (plannerType)
    {
        case 0: {
            auto rrtconnect = new og::RRTConnect(si);
            planner.reset(rrtconnect);
            // planner(std::make_shared<og::RRTConnect>(si));
            break;
        }
        case 1: {
            auto rrtstar = new og::RRTstar(si);
            planner.reset(rrtstar);
            break;
        }
        default: {
            std::cout << "Wrong planner!" << std::endl;
            break;
        }
    }
    // set optimization objective
    pdef->setOptimizationObjective(getPathLengthObjective(si));

    // set the problem we are truing to solve for the planner
    planner->setProblemDefinition(pdef);

    //perform setup steps
    planner->setup();

    // print the settings for this space
    si->printSettings(std::cout);

    // print the problem settings
    pdef->print(std::cout);

    ob::PlannerStatus solved = planner->ob::Planner::solve(timelimit);

    if (solved) 
    {
        std::cout << "found solution!" << std::endl;
        auto path = pdef->getSolutionPath()->as<og::PathGeometric>();
        path->print(std::cout);
        path->interpolate();

        // stream the solution to the output file
        std::ofstream out(outputFile);
        out << "result:" << std::endl;
        out << "  - states:" << std::endl;
        
        for (size_t i = 0; i < path->getStateCount(); ++i) 
        {
            const auto state = path->getState(i);
            std::vector<double> statevec;
            si->getStateSpace()->copyToReals(statevec, state);
            out << "      - [";
            for (size_t i = 0; i < statevec.size(); ++i) 
            {
                out << statevec[i];
                if (i < statevec.size() - 1) {
                    out << ",";
                }
            }
             out << "]" << std::endl;
        }
    }
    else {
        std::cout << "No solution found" << std::endl;
    }
}
int main(int argc, char* argv[])
{

    std::cout << "OMPL version: "<< OMPL_VERSION <<std::endl;

    namespace po = boost::program_options;
    
    // Declare the supported options.
    po::options_description desc("Allowed options");
    std::string inputFile;
    std::string outputFile;

    desc.add_options()
        ("help", "produce help message")
        ("input,i", po::value<std::string>(&inputFile)->required(), "input file (yaml)")
        ("output,o", po::value<std::string>(&outputFile)->required(), "output file (yaml)");

    try {
        po::variables_map vm;
        po::store(po::parse_command_line(argc, argv, desc), vm);
        po::notify(vm);
        if (vm.count("help") != 0u) {
                    std::cout << desc << "\n";
            return 0;

        }
    } catch (po::error& e) {
        std::cerr << e.what() << std::endl << std::endl;
        std::cerr << desc << std::endl;
        return 1;
    }

    YAML::Node configFile = YAML::LoadFile(inputFile);
    
    // planner type
    const size_t& plannerType = configFile["plannerType"].as<double>();
    // timelimit to find a solution
    const float& timelimit = configFile["timelimit"].as<float>();
    
    // start and goal states from config
    const auto& startSt = configFile["payload"]["start"];
    std::vector<double> startvec;
    for (const auto& start : startSt) {
        startvec.push_back(start.as<double>());
    }
    
    const auto& goalSt = configFile["payload"]["goal"];
    std::vector<double> goalvec;
    for (const auto& goal : goalSt) {
        goalvec.push_back(goal.as<double>());
    }
    
    // extract environment bounds from config 
    const auto& envmincfg = configFile["environment"]["min"];    
    std::vector<double> envmin;
    for (const auto& min : envmincfg) {
        envmin.push_back(min.as<double>());
    }

    const auto& envmaxcfg = configFile["environment"]["max"];
    std::vector<double> envmax;
    for (const auto& max : envmaxcfg) {
        envmax.push_back(max.as<double>());
    } 

    payloadPlan(startvec, goalvec, envmin, envmax, plannerType, timelimit, outputFile);

    std::cout << std::endl << std::endl;


    return 0;
}