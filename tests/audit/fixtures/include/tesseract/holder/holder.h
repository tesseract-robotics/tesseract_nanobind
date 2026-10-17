// Audit contract fixture: a type-erased holder in the shape of tesseract::common::AnyPoly
// (any_poly.h: `AnyPoly(const AnyInterface&)` and the two constrained forwarding ctor templates).
#pragma once
#include <memory>
#include <type_traits>

namespace tesseract::holder
{
struct AnyHolderInterface  // abstract: the class itself is a gap row
{
  virtual ~AnyHolderInterface() = default;
  virtual int get() const = 0;
};

class AnyHolder
{
public:
  AnyHolder();
  AnyHolder(const AnyHolderInterface& impl);  // stays a gap row: not a template
  // No bound arity-1 constructor on purpose: an unknown template-parameter type falls back to
  // arity alone, so a bound `AnyHolder(int)` would "cover" the forwarding templates below.

  // Forwarding ctor templates, constrained like AnyPoly's: template-instance-name.
  template <typename T,
            typename std::enable_if_t<!std::is_base_of_v<AnyHolderInterface, T> &&
                                          !std::is_same_v<std::decay_t<T>, AnyHolder>,
                                      bool> = true>
  AnyHolder(T&& value);

  template <typename T,
            typename std::enable_if_t<!std::is_base_of_v<AnyHolderInterface, T> &&
                                          !std::is_same_v<std::decay_t<T>, AnyHolder>,
                                      bool> = true>
  AnyHolder(const T& value);

  template <typename T>
  AnyHolder(const T& value, int tag);  // not all template parameters: stays a gap row

private:
  std::unique_ptr<AnyHolderInterface> impl_;
};

// Not forwarding: the parameter names the class's template parameter, or points to the ctor's.
template <typename T>
class AnyBox
{
public:
  template <typename U>
  AnyBox(const T& value);  // T is AnyBox's, not the ctor template's own

  template <typename U>
  AnyBox(U* value);  // a pointer, not a reference

private:
  T value_;
};
}  // namespace tesseract::holder
