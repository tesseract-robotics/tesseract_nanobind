// Audit contract fixture (#256 T5): stub base classes vs the bound C++ bases.
#pragma once

namespace tesseract::bases
{
struct Jogger  // bound base
{
  virtual ~Jogger() = default;
  virtual void go() = 0;
};

struct SlowJogger final : Jogger  // stub omits the Jogger base: gap SlowJogger(Jogger) (T5-1)
{
  SlowJogger();
  void go() override;
};

struct Middle : Jogger  // unbound intermediate
{
};

struct DeepJogger final : Middle  // stub base Jogger is the nearest bound ancestor: no row (T5-2)
{
  DeepJogger();
  void go() override;
};

struct Loner  // stub claims a base C++ does not have: deviation Loner(Jogger) (T5-3)
{
  Loner();
};

struct Hidden : private Jogger  // private base: Python sees no base, no row
{
  Hidden();
  void go() override;
};

struct Tagged : Jogger  // stub names its base: no row
{
  Tagged();
  void go() override;
};

struct RemoteJogger : Jogger  // stub base is bases_remote's Jogger: same class only if the index agrees
{
  RemoteJogger();
  void go() override;
};

template <typename T>
struct Planner : Jogger  // Planner<double> bound as PlannerD without its base: gap PlannerD(Jogger)
{
  void go() override;
  T value;
};
struct SubPlanner : Planner<double>  // stub base PlannerD resolves through the own index: no row
{
  SubPlanner();
};
}  // namespace tesseract::bases
