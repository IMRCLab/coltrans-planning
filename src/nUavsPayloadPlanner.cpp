// Planners
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>

#include <ompl/config.h>
#include <fcl/fcl.h>
#include <yaml-cpp/yaml.h>

#include <boost/program_options.hpp>
#include <iostream>
#include <fstream>
#include <helper.hpp>
#include <optimObj.hpp>
#include <robots.hpp>

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

    auto fullstate = state->as<StateSpace::StateType>();
    Eigen::Vector3f attachmentpoint(0, 0.1, 0);
    const auto cablepos = fullstate->getCablePos(1, attachmentpoint, 0.5);
    // const auto payloadrot = fullstate->getPayloadRot();
    // std::cout << payloadrot << std::endl;
    // std::vector<double> position;
 
    // exit(3);
    // return a value that is always true but uses the two variables we define, so we avoid compiler warnings
    return (const void*)rot != (const void*)pos;
}



void cablesPayloadPlanner(const plannerSettings& cfg, std::string &outputFile)
{
    nCablesPayload nCablesPayload(cfg);   
    nCablesPayload.addRobotParts(cfg);
    Obstacles Obstacles(cfg.env);

    // construct an instance of space information from this state
    auto si = nCablesPayload.si;

    // set state validity checking for this space
    si->setStateValidityChecker(isStateValid);
    
     // create a problem instance
    auto pdef(std::make_shared<ob::ProblemDefinition>(si));

    // create and set a start state
    auto startState = si->allocState();
    si->getStateSpace()->copyFromReals(startState, eigentoStd(cfg.start));
    si->enforceBounds(startState);
    pdef->addStartState(startState);
    si->freeState(startState);
    
    // create and set a goal state
    auto goalState = si->allocState();
    si->getStateSpace()->copyFromReals(goalState, eigentoStd(cfg.goal));
    si->enforceBounds(goalState);
    pdef->setGoalState(goalState);
    si->freeState(goalState);

    std::shared_ptr<ob::Planner> planner;
    //create planner
    if (cfg.plannerType == "rrtconnect") 
    {
        auto rrtconnect = new og::RRTConnect(si);
        planner.reset(rrtconnect);
    } else if (cfg.plannerType == "rrtstar") 
    {
        auto rrtstar = new og::RRTstar(si);
        planner.reset(rrtstar);
    } else 
    {
        std::cout << "Wrong Planner!" << std::endl;
        exit(3);
    }
    // pdef->setOptimizationObjective(getPathLengthObjective(si));
    auto objectCable(std::make_shared<minCableObjective>(si, cfg.desiredCableStates));
    objectCable->setCostThreshold(ob::Cost(1.0));
    pdef->setOptimizationObjective(objectCable);

    // set the problem we are truing to solve for the planner
    planner->setProblemDefinition(pdef);

    //perform setup steps
    planner->setup();

    // print the settings for this space
    si->printSettings(std::cout);

    // print the problem settings
    pdef->print(std::cout);

    ob::PlannerStatus solved = planner->ob::Planner::solve(cfg.timelimit);

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
                if (i < statevec.size() - 1) 
                {
                    out << ",";
                }
            }
             out << "]" << std::endl;
        }
        // stream the  cable lengths to the output file
        out<< "  - cablelengths:" << std::endl;
        out<<"      - [";
               
        for (size_t i = 0; i < cfg.cableslength.size(); ++i) 
        {
            out << cfg.cableslength[i];
            if (i < cfg.cableslength.size()-1) 
            {
                out << ",";
            }
        }
        out << "]" <<std::endl;    
        // stream the cable attachment points to the output file
        out<< "  - cablepoints:" << std::endl;
        out<<"      - [";
       
        for (int i = 0; i < cfg.attachmentpoints.size(); ++i) 
        {
            out << cfg.attachmentpoints[i];
            if (i < cfg.attachmentpoints.size()-1) 
            {
                out << ",";
            }
        }
        out << "]" <<std::endl;    

    }
    else 
    {
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
    plannerSettings cfg;
    
    // planner type
    cfg.plannerType = configFile["plannerType"].as<std::string>();
    // timelimit to find a solution
    cfg.timelimit = configFile["timelimit"].as<float>();

    // payload shape
    cfg.payloadShape = configFile["payload"]["shape"]["type"].as<std::string>();

    // number of uavs
    cfg.numofcables = configFile["numofcables"].as<size_t>();

    // start and goal states from config
    cfg.start = yamltoEigen(configFile["payload"]["start"]);
    
    cfg.goal = yamltoEigen(configFile["payload"]["goal"]);
   
    // extract environment bounds from config 
    cfg.envmin = yamltovec(configFile["environment"]["min"]);

    cfg.envmax = yamltovec(configFile["environment"]["max"]);

    // minimum elevation
    cfg.cablemin = yamltovec(configFile["cables"]["min"]);
    // maximum elevation
    cfg.cablemax = yamltovec(configFile["cables"]["max"]);

    
    cfg.cableslength = yamltovec(configFile["cables"]["lengths"]);

    cfg.attachmentpoints = yamltoEigen(configFile["cables"]["attachmentpoints"]);
    
    cfg.desiredCableStates = yamltovec(configFile["cables"]["desired"]);

    // Extract obstacles
    cfg.env = configFile["environment"]["obstacles"];

    cablesPayloadPlanner(cfg, outputFile);

    std::cout << std::endl << std::endl;

}