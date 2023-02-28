// Spaces
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>

// Objectives
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
// Planners
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

void cablesPayloadPlanner(const std::vector<double>* start, const std::vector<double>* goal,
                 const std::vector<double>* envmin, const std::vector<double>* envmax,
                 const std::vector<double>* cablemin, const std::vector<double>* cablemax,
                 const std::vector<double>* cableslength, const std::vector<double>* cablespoints,
                 const std::string &plannerType, const float &numofuavs, const float &timelimit, std::string outputFile)
{
    // construct compound state space SE(3) X (R^2)^2
    // SE3 for the payload
    auto payload(std::make_shared<ob::SE3StateSpace>());
    // orientation of the two cables: az1, elv1, az2, elv2
    auto cables(std::make_shared<ob::RealVectorStateSpace>(2*numofuavs));
   
    ob::RealVectorBounds payloadbounds(3);
    // prepare the payload position bounds
    for (size_t i=0; i < envmin->size(); ++i) {
        payloadbounds.setLow(i, envmin->operator[](i));
        payloadbounds.setHigh(i, envmax->operator[](i));
    }
    payload->setBounds(payloadbounds);
    
    ob::RealVectorBounds cablebounds(2*numofuavs);
    // prepare the cable elevation bounds
    for (size_t i=0; i < cablemin->size(); ++i) {
        cablebounds.setLow(i, cablemin->operator[](i));
        cablebounds.setHigh(i, cablemax->operator[](i));
    }
    cables->setBounds(cablebounds);
    auto space = payload +  cables;

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
    si->getStateSpace()->copyFromReals(startState, *start);
    si->enforceBounds(startState);
    pdef->addStartState(startState);
    si->freeState(startState);
    
    // create and set a goal state
    auto goalState = si->allocState();
    si->getStateSpace()->copyFromReals(goalState, *goal);
    si->enforceBounds(goalState);
    pdef->setGoalState(goalState);
    si->freeState(goalState);

    std::shared_ptr<ob::Planner> planner;
    //create planner
    if (plannerType == "rrtconnect") {
        auto rrtconnect = new og::RRTConnect(si);
        planner.reset(rrtconnect);
    } else if (plannerType == "rrtstar") {
        auto rrtstar = new og::RRTstar(si);
        planner.reset(rrtstar);
    } else {
        std::cout << "Wrong Planner!" << std::endl;
        exit(3);
    }


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
        // stream the  cable lengths to the output file
        out<< "  - cablelengths:" << std::endl;
        out<<"      - [";
               
        for (size_t i = 0; i < cableslength->size(); ++i) {
       
            out << cableslength->operator[](i);
       
            if (i < cableslength->size()-1) {
                out << ",";
            }
        }
        out << "]" <<std::endl;    
        // stream the cable attachment points to the output file
        out<< "  - cablepoints:" << std::endl;
        out<<"      - [";
       
        for (size_t i = 0; i < cablespoints->size(); ++i) {
       
            out << cablespoints->operator[](i);
       
            if (i < cablespoints->size()-1) {
                out << ",";
            }
        }
        out << "]" <<std::endl;    

    }
    else {
        std::cout << "No solution found" << std::endl;
    }

}

void yamltovec(std::vector<double> *vec, const YAML::Node &yamlvec) {
    // helper function to convert yaml vec to  std::vec
    for (const auto& i : yamlvec) {
        vec->push_back(i.as<double>());
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
    const std::string& plannerType = configFile["plannerType"].as<std::string>();
    // timelimit to find a solution
    const float& timelimit = configFile["timelimit"].as<float>();
    
    // number of uavs
    const float& numofuavs = configFile["numofuavs"].as<float>();

    // start and goal states from config
    const auto &startSt = configFile["payload"]["start"];
    std::vector<double> *startvec = new std::vector<double>();
    yamltovec(startvec, startSt);
    
    const auto& goalSt = configFile["payload"]["goal"];
    std::vector<double> *goalvec = new std::vector<double>();
    yamltovec(goalvec, goalSt);
   
    // extract environment bounds from config 
    const auto& envmincfg = configFile["environment"]["min"];    
    std::vector<double> *envmin = new std::vector<double>();
    yamltovec(envmin, envmincfg);

    const auto& envmaxcfg = configFile["environment"]["max"];
    std::vector<double> *envmax = new std::vector<double>();
    yamltovec(envmax, envmaxcfg);

    // minimum elevation
    const auto& cablesmincfg = configFile["cables"]["min"];
    std::vector<double> *cablesmin = new std::vector<double>();
    yamltovec(cablesmin, cablesmincfg);
    // maximum elevation
    const auto& cablesmaxcfg = configFile["cables"]["max"];
    std::vector<double> *cablesmax = new std::vector<double>();
    yamltovec(cablesmax, cablesmaxcfg);

    
    const auto& cableslengthcfg = configFile["cables"]["lengths"];
    std::vector<double> *cableslength = new std::vector<double>();
    yamltovec(cableslength, cableslengthcfg);

    const auto& cablespointscfg = configFile["cables"]["attachmentpoints"];
    std::vector<double> *cablespoints = new std::vector<double>();
    yamltovec(cablespoints, cablespointscfg);
  

    cablesPayloadPlanner(startvec, goalvec,
                         envmin, envmax, 
                         cablesmin, cablesmax, 
                        cableslength, cablespoints, plannerType, numofuavs, timelimit, outputFile);

    std::cout << std::endl << std::endl;

}