#pragma once
#include <memory>
namespace vendor
{
struct SpaceInformation
{
};
struct Planner
{
};
using SpaceInformationPtr = std::shared_ptr<SpaceInformation>;
using PlannerPtr = std::shared_ptr<Planner>;
struct State
{
};
}  // namespace vendor
