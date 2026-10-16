// Audit contract fixture: a type-erased holder in the shape of tesseract::common::AnyPoly.
#pragma once
#include <memory>

namespace tesseract::holder
{
struct HolderInterface  // abstract: the class itself is a gap row
{
  virtual ~HolderInterface() = default;
  virtual int get() const = 0;
};

class Holder
{
public:
  Holder();
  Holder(const HolderInterface& impl);  // stays a gap row: not a template
  // No bound arity-1 constructor on purpose: an unknown template-parameter type falls back to
  // arity alone, so a bound `Holder(int)` would "cover" the forwarding template below.

  template <typename T>
  Holder(T&& value);  // forwarding ctor template: template-instance-name

  template <typename T>
  Holder(const T& value, int tag);  // not all template parameters: stays a gap row

private:
  std::unique_ptr<HolderInterface> impl_;
};
}  // namespace tesseract::holder
