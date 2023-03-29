#include <fstream>
#include <iostream>
#include <algorithm>

#include <yaml-cpp/yaml.h>

#include <boost/program_options.hpp>

// #include <fcl/collision.h>
// #include <fcl/collision_object.h>
// #include <fcl/broadphase/broadphase.h>
#include <fcl/fcl.h>

Eigen::Vector3f getunitvec(float az, float el)
{    
    // azimuth and elevation --> unit vec
    // source https://math.stackexchange.com/questions/1150232/finding-the-unit-direction-vector-given-azimuth-and-elevation
    Eigen::Vector3f unitvec(cos(az)*cos(el), sin(az)*cos(el), sin(el));
    return unitvec;
}


int main(int argc, char* argv[]) {
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

  YAML::Node test_fcl = YAML::LoadFile(inputFile);
  // Load obstacles
  std::vector<fcl::CollisionObjectf *> obstacles;
  for (const auto& obs : test_fcl["environment"]["obstacles"]) {
    if (obs["type"].as<std::string>() == "cylinder") {
      std::shared_ptr<fcl::CollisionGeometryf> geom;
      const auto radius = obs["radius"].as<float>();
      const auto height = obs["height"].as<float>();
      
      const auto& rotation = obs["rotation"];
      
      Eigen::Vector3f unitvec = getunitvec(rotation[0].as<float>(), rotation[1].as<float>());
      std::cout << "unit vec: \n" << unitvec << "\n" << std::endl; 
      Eigen::Vector3f basevec(0,0,1);
      Eigen::Quaternionf quat = Eigen::Quaternionf::FromTwoVectors(basevec, unitvec);
      std::cout << "quat: \n" << quat << "\n" << std::endl; 
      
      geom.reset(new fcl::Cylinderf(radius, height));
      const auto& center = obs["center"];
      auto co = new fcl::CollisionObjectf(geom);
      std::cout << "pos obs:" << "\n" << center << std::endl;
      fcl::Transform3f transform;
      transform = Eigen::Translation<float, 3>(fcl::Vector3f(center[0].as<float>(), center[1].as<float>(), center[2].as<float>()));
      transform.rotate(quat);
      
      co->setTranslation(transform.translation());
      co->setRotation(transform.rotation());
      co->computeAABB();
      obstacles.push_back(co);
    } else if (obs["type"].as<std::string>() == "sphere") {
      std::shared_ptr<fcl::CollisionGeometryf> geom;
      const auto radius = obs["radius"].as<float>();

      geom.reset(new fcl::Spheref(radius));
      const auto& center = obs["center"];
      auto co = new fcl::CollisionObjectf(geom);
      std::cout << "pos obs:" << "\n" << center << std::endl;
      fcl::Transform3f transform;
      transform = Eigen::Translation<float, 3>(fcl::Vector3f(center[0].as<float>(), center[1].as<float>(), center[2].as<float>()));

      co->setTranslation(transform.translation());
      co->computeAABB();
      obstacles.push_back(co);
    }
     else { 
      throw std::runtime_error("Unknown obstacle type!");
    }
  }
  fcl::BroadPhaseCollisionManagerf* envObs = new fcl::DynamicAABBTreeCollisionManagerf();
  envObs->registerObjects(obstacles);
  envObs->setup();


  // stream out the obstacles for visualizatation
  std::ofstream out(outputFile);
  out << "obstacles:" << std::endl;
  YAML::Node env = test_fcl["environment"]["obstacles"];
  for(size_t i = 0; i < env.size(); ++i) {
      YAML::Node obstacle = env[i];
      out<<"  - type: "<<obstacle["type"] <<"\n";   
      out<<"    center: "<< obstacle["center"]<<"\n";   
      out<<"    rotation: "<< obstacle["rotation"]<<"\n";   
      out<<"    radius: "<< obstacle["radius"] <<"\n";   
    if (obstacle["type"].as<std::string>() == "cylinder") {
      out<<"    height: "<< obstacle["height"] <<"\n";   
    }
  }
  YAML::Node payloadYaml = test_fcl["payload"];
  out << "payload:" << std::endl;
  out << " center: " << payloadYaml["center"]<<"\n";
  out << " type: " << payloadYaml["type"]<<"\n";
  out << " radius: " << payloadYaml["radius"]<<"\n";

  //Load Payload
  std::shared_ptr<fcl::CollisionObjectf> payload;
  if (test_fcl["payload"]["type"].as<std::string>() == "sphere") {
    std::shared_ptr<fcl::CollisionGeometryf> geomLoad;
    const auto payloadRadius = test_fcl["payload"]["radius"].as<float>();
    geomLoad.reset(new fcl::Spheref(payloadRadius));
    const auto& centerLoad = test_fcl["payload"]["center"];
    payload.reset(new fcl::CollisionObjectf(geomLoad));
    payload->setTranslation(fcl::Vector3f(centerLoad[0].as<float>(), centerLoad[1].as<float>(), centerLoad[2].as<float>()));
    payload->computeAABB();
  } else {
    throw std::runtime_error("Unknown payload type!");
  }
  


  fcl::DefaultCollisionData<float> collision_data;
  envObs->collide(payload.get(), &collision_data, fcl::DefaultCollisionFunction<float>);
  
  std::cout << collision_data.result.isCollision() << std::endl;
  std::cout<<"robot-obstacle collision: "<<collision_data.result.isCollision()<<std::endl;

  fcl::DefaultCollisionData<float> inter_collision_data;
  envObs->collide(&inter_collision_data, fcl::DefaultCollisionFunction<float>);
  std::cout <<  "inter-collision: "<<inter_collision_data.result.isCollision() << std::endl;
  
  return 0;
}