#include <chrono>
#include <iostream>
#include <fstream>

// Planners
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/sst/SST.h>
#include <ompl/config.h>
#include <fcl/fcl.h>
#include <yaml-cpp/yaml.h>

#include <boost/program_options.hpp>

#include <helper.hpp>
#include "optimObj.h"
#include "robots.h"
#include "fclStateValidityChecker.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

void cablesPayloadPlanner(const plannerSettings& cfg, std::string &outputFile, std::string &statsFile)
{
    std::shared_ptr<RobotsWithPayload> nCablesPayload = create_robots(cfg);   
    std::shared_ptr<Obstacles> Obstacles = create_obs(cfg.env);

    // construct an instance of space information from this state
    auto si = nCablesPayload->si;

    // set state validity checking for this space
    auto stateValidityChecker(std::make_shared<fclStateValidityChecker>(si, nCablesPayload, Obstacles, cfg));
    si->setStateValidityChecker(stateValidityChecker);

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

    } else if (cfg.plannerType == "sst") 
    {
        auto sst = new og::SST(si);
        planner.reset(sst);
    } else 
    {
        std::cout << "Wrong Planner!" << std::endl;
        exit(3);
    }
    auto objectCable(std::make_shared<minCableObjective>(si));
    objectCable->setCostThreshold(ob::Cost(1.0));
    pdef->setOptimizationObjective(objectCable);

    // empty stats file
    std::ofstream stats(statsFile);
    stats << "stats:" << std::endl;

    auto start = std::chrono::steady_clock::now();

    pdef->setIntermediateSolutionCallback(
        [&start, &stats](const ob::Planner *, const std::vector<const ob::State *> &, const ob::Cost cost)
        {
            auto now = std::chrono::steady_clock::now();
            double t = std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count();
            stats << "  - t: " << t/1000.0f << std::endl;
            stats << "    cost: " << cost.value() << std::endl;
            std::cout << "Intermediate solution! " << cost.value() << " " << t/1000.0f << std::endl;
        });
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
        path->interpolate(cfg.interpolate);

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
               
        for (size_t i = 0; i < cfg.cablelengthVec.size(); ++i) 
        {
            out << cfg.cablelengthVec[i];
            if (i < cfg.cablelengthVec.size()-1) 
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
        //stream out obstacle types, shapes and positions:
        out<< "  - obstacles:" << std::endl;
        for(size_t i = 0; i < cfg.env.size(); ++i) {
            YAML::Node obs = cfg.env[i];
            out<<"      - type: "<<obs["type"] <<"\n";   
            out<<"        center: "<< obs["center"]<<"\n";   
            out<<"        radius: "<< obs["radius"] <<"\n";   
            if (obs["type"].as<std::string>() == "cylinder") {
                out<<"        height: "<< obs["height"] <<"\n";   
            }
        }
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
    std::string statsFile;

    desc.add_options()
        ("help", "produce help message")
        ("input,i", po::value<std::string>(&inputFile)->required(), "input file (yaml)")
        ("output,o", po::value<std::string>(&outputFile)->required(), "output file (yaml)")
        ("stats", po::value<std::string>(&statsFile)->default_value("ompl_stats.yaml"), "output file (yaml)");

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

    
    cfg.cablelengthVec = yamltovec(configFile["cables"]["lengths"]);

    cfg.attachmentpoints = yamltoEigen(configFile["cables"]["attachmentpoints"]);
    
    cfg.interpolate = configFile["interpolate"].as<int>();
    if (cfg.payloadShape == "rod" || cfg.payloadShape == "triangle") {
        cfg.angle_min = configFile["payload"]["angle_min"].as<float>();
        cfg.angle_max = configFile["payload"]["angle_max"].as<float>();
    }
    // Extract obstacles
    cfg.env = configFile["environment"]["obstacles"];

    cablesPayloadPlanner(cfg, outputFile, statsFile);

    std::cout << std::endl << std::endl;

}
