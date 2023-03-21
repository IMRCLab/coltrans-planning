#pragma once

#include "robots.h"

class fclStateValidityChecker
  : public ompl::base::StateValidityChecker
{
public:
  fclStateValidityChecker(
      ompl::base::SpaceInformationPtr si,
      std::shared_ptr<RobotsWithPayload> robots,
      std::shared_ptr<Obstacles> obstacles,
      const Eigen::VectorXf& attachmentpoints,
      const std::vector<double>& cablelengthVec);

  bool isValid(const ompl::base::State* state) const override;

private:
  std::shared_ptr<RobotsWithPayload> robots_;
  std::shared_ptr<Obstacles> obstacles_;
  const Eigen::VectorXf attachmentpoints_;
  std::vector<double> cablelengthVec_;
};
