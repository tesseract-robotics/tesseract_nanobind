// Audit contract fixture (#256 T3): classes that identity_bindings.cpp binds under Python names.
#pragma once

namespace tesseract::identity
{
struct LongWidgetName  // bound as Widget: a rename
{
  struct Part  // bound on the Widget class object: a nested class, skipped
  {
  };
  int size() const;
};

template <typename T>
struct Gadget  // Gadget<double> bound as GadgetD: a template instance
{
  T value;
};

enum class Color  // nb::enum_
{
  RED
};

struct Derived : LongWidgetName  // a bound base in the template arguments
{
};
}  // namespace tesseract::identity
