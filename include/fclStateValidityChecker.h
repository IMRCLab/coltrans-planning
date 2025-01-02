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
      const plannerSettings& cfg);

  bool isValid(const ompl::base::State* state) const override;

protected:
  std::shared_ptr<RobotsWithPayload> robots_;
  std::shared_ptr<Obstacles> obstacles_;
  const plannerSettings cfg_;
  Eigen::Vector3f qvrot(const Eigen::Quaternionf &q, const Eigen::Vector3f &v) const;
};


class fclUnicyclesStateValidityChecker

  : public ompl::base::StateValidityChecker
{
public:
  fclUnicyclesStateValidityChecker(
      ompl::base::SpaceInformationPtr si,
      std::shared_ptr<UnicyclesWithRods> robots,
      std::shared_ptr<Obstacles> obstacles,
      const unicycleSettings& cfg);


  bool isValid(const ompl::base::State* state) const override;

protected:
  std::shared_ptr<UnicyclesWithRods> robots_;
  std::shared_ptr<Obstacles> obstacles_;
  const unicycleSettings cfg_;
};
