#include "helper.h"

bool startsWith(const std::string &str, const std::string &prefix) {
  return str.size() >= prefix.size() &&
         str.compare(0, prefix.size(), prefix) == 0;
}

std::vector<double> yamltovec(const YAML::Node &yamlvec) 
{
    std::vector<double> vec;
    // helper function to convert yaml vec to  std::vec
    for (const auto& i : yamlvec) {
        vec.push_back(i.as<double>());
    }
    return vec;
}

Eigen::VectorXf yamltoEigen(const YAML::Node &yamlvec)
{
    Eigen::VectorXf vec(yamlvec.size());
    for (size_t i = 0; i < yamlvec.size();++i) 
    {
        vec(i) = yamlvec[i].as<double>();
    }
    return vec;
}

Eigen::VectorXf stdtoEigen(const std::vector<double>& stdvec)
{
    Eigen::VectorXf eigenvec(stdvec.size());
    for (size_t i = 0; i < stdvec.size(); ++i)
    {
        eigenvec(i) = stdvec[i];
    }
    // Eigen::VectorXd eigenvec = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(stdvec.data(), stdvec.size());
    return eigenvec;
}

std::vector<double> eigentoStd(const Eigen::VectorXf &eigenvec)
{
    std::vector<double> stdvec;
    for (const auto& i : eigenvec)
    {
        stdvec.push_back(i);
    }
    return stdvec;
}
