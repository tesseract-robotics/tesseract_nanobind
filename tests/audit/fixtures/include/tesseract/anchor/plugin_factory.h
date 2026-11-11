// Audit contract fixture: a plugin factory interface, named as one in the rule tables
// (`plugin-author-factory-interface`).
#pragma once

namespace tesseract::anchor
{
class InvKinFactory
{
public:
  virtual ~InvKinFactory() = default;
  virtual int create() const = 0;
};
}  // namespace tesseract::anchor
