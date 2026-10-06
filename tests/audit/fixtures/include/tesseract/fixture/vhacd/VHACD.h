#pragma once

// Stand-in for a vendored third-party single-header library (V-HACD): included directly
// by fixture_bindings.cpp, never Python API.
namespace VHACD
{
class IVHACD
{
public:
  virtual ~IVHACD() = default;
  virtual bool Compute() = 0;
};

IVHACD* CreateVHACD();
}  // namespace VHACD
