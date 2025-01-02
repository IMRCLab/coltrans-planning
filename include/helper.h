#pragma once
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>


std::vector<double> yamltovec(const YAML::Node &yamlvec);

Eigen::VectorXf yamltoEigen(const YAML::Node &yamlvec);

Eigen::VectorXf stdtoEigen(const std::vector<double>& stdvec);

std::vector<double> eigentoStd(const Eigen::VectorXf &eigenvec);

bool startsWith(const std::string &str, const std::string &prefix);