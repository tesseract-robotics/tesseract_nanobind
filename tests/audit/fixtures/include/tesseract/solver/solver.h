#pragma once
#include <memory>
#include <thirdparty_fixture/settings.h>

namespace tesseract::solver
{
enum class Status
{
  kRunning,        // bound RUNNING: mechanical
  kQPSolveFailed,  // bound QP_SOLVE_FAILED: mechanical, acronym run
  kConverged       // bound NLP_CONVERGED: not mechanical, stays a gap and a deviation
};

struct Shift
{
  void operator+=(double s);
  void operator-=(double s);
};

class Engine
{
public:
  static void setDefaults(vendor::Settings& settings);  // third-party param: absent ok
  std::unique_ptr<vendor::Settings> settings_;          // third-party field: absent ok
  bool warmStart(const Shift& shift);                   // bound: covered
  bool tune(const vendor::Settings& settings, int n);   // bound under the same name
};
}  // namespace tesseract::solver
