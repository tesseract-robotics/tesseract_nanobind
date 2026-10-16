// Audit contract fixture (#256 T5): stub base classes vs the bound C++ bases.
#pragma once

namespace tesseract::bases
{
struct Runner  // bound base
{
  virtual ~Runner() = default;
  virtual void go() = 0;
};

struct SlowRunner final : Runner  // stub omits the Runner base: gap SlowRunner(Runner) (T5-1)
{
  SlowRunner();
  void go() override;
};

struct Middle : Runner  // unbound intermediate
{
};

struct DeepRunner final : Middle  // stub base Runner is the nearest bound ancestor: no row (T5-2)
{
  DeepRunner();
  void go() override;
};

struct Loner  // stub claims a base C++ does not have: deviation Loner(Runner) (T5-3)
{
  Loner();
};

struct Hidden : private Runner  // private base: Python sees no base, no row
{
  Hidden();
  void go() override;
};

struct Tagged : Runner  // stub names its base: no row
{
  Tagged();
  void go() override;
};

template <typename T>
struct Planner : Runner  // Planner<double> bound as PlannerD without its base: gap PlannerD(Runner)
{
  void go() override;
  T value;
};
}  // namespace tesseract::bases
