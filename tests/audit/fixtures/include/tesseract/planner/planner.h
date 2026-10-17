#pragma once
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <Eigen/Core>
#include <yaml-cpp/yaml.h>
#include <tesseract/planner/internal/term.h>
#include <thirdparty_fixture/planning.h>

// Only forward-declared, as tesseract's OMPL headers declare `ompl::base::Planner` (R13).
namespace vendorfwd
{
class Planner;
}  // namespace vendorfwd

namespace tesseract::planner
{
struct PluginFactory
{
};

// T1: YAML ctor with a bound YAML-text sibling: yaml-node accepted.
class Loader
{
public:
  Loader();
  Loader(const YAML::Node& config);
  Loader(const std::string& config);
};

// T1: YAML ctor, no sibling (the planner-profile shape): a gap.
class Profile
{
public:
  Profile();
  Profile(const YAML::Node& config, const PluginFactory& plugin_factory);
};

// T1: sibling exists in C++ but is unbound: the YAML overload is a gap.
class UnboundSibling
{
public:
  UnboundSibling();
  UnboundSibling(const YAML::Node& config);
  UnboundSibling(const std::string& config);
};

// T1: a bound text sibling that differs at another position is no sibling: a gap.
class Mismatched
{
public:
  Mismatched();
  Mismatched(const YAML::Node& config, int n);
  Mismatched(const std::string& config, double x);
};

// T1: a YAML-text sibling on a base class (`from_yaml`, C6 6a) does not cover a derived
// class's YAML ctor: ctors are never inherited, and `from_yaml` is per profile class.
class BaseProfile
{
public:
  BaseProfile();
};

class DerivedProfile : public BaseProfile
{
public:
  DerivedProfile();
  DerivedProfile(const YAML::Node& config);
};

// C6a: YAML ctor whose Python form is a static from_yaml(text).
class TextProfile
{
public:
  TextProfile();
  TextProfile(const YAML::Node& config, const PluginFactory& plugin_factory);
};

// C6a negative: from_yaml takes no text, so it is no YAML-text sibling.
class IntYamlProfile
{
public:
  IntYamlProfile();
  IntYamlProfile(const YAML::Node& config);
};

// C6a negative: from_yaml is an instance method, so it is no YAML-text sibling.
class MethodYamlProfile
{
public:
  MethodYamlProfile();
  MethodYamlProfile(const YAML::Node& config);
};

// C6a negative: from_yaml returns another class, so it does not construct this one.
class ForeignYamlProfile
{
public:
  ForeignYamlProfile();
  ForeignYamlProfile(const YAML::Node& config);
};

// Task 4 (R13): third-party and not-API types in parameters, returns and fields.
class Configurator
{
public:
  Configurator();
  vendor::PlannerPtr create(vendor::SpaceInformationPtr si) const;  // third-party param+return
  int getType() const;                                              // bound: covered
};

// Task 4 (R13): a forward-declared third-party type has no third-party header to match.
class ForwardHooks
{
public:
  ForwardHooks();
  std::shared_ptr<vendorfwd::Planner> create() const;
};

using StateExtractor = std::function<Eigen::VectorXd(const vendor::State*)>;

class Hooks
{
public:
  Hooks();
  StateExtractor createStateExtractor() const;                   // std::function over a vendor type
  std::shared_ptr<internal::Term> createTerm() const;            // not-API return
  std::vector<std::function<void(internal::Term*)>> callbacks;  // not-API, inside std::function
  std::function<void(int)> on_step;                              // no unbound type: stays a gap
  Configurator makeConfigurator() const;                         // owned, audited type: stays a gap
};

// T13: constructor template of a class template.
template <class T>
struct CtorTemplateBox
{
  CtorTemplateBox();
  template <class U>
  explicit CtorTemplateBox(const U& u);
};
using CtorTemplateBoxD = CtorTemplateBox<double>;

// T12: no operator= is ever a row; other operators keep theirs.
struct AssignHolder
{
  AssignHolder();
  AssignHolder& operator=(int value);  // converting assignment: never a row
  template <class T>
  AssignHolder& operator=(T&& value);                     // templated assignment: never a row
  bool operator==(const AssignHolder& rhs) const;          // still a row: no __eq__ in the stub
  AssignHolder operator+(const AssignHolder& rhs) const;  // still a row: no __add__ in the stub
};
}  // namespace tesseract::planner
